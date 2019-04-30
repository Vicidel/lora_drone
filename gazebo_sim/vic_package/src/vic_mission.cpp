/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

// standard libraries
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

// libraries for Vector3f
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation

// ROS libraries
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>

// for server communication
#include "server_app.h"


// define the received MAVROS messages
mavros_msgs::State current_state;               // drone state (for OFFBOARD mode)
geometry_msgs::PoseStamped  est_local_pos;      // local position (x, y, z)


// functions definitions
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos);
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
Vector3f parse_WP_from_answer(std::string answer_string, Vector3f pos_current_goal);
float parse_hover_time_from_answer(std::string answer_string);


// main
int main(int argc, char **argv){    
    //printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    // ROS initialization
    ros::init(argc, argv, "vic_mission");
    ros::NodeHandle nh;

    // subscribes to topics 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    ros::Subscriber est_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, est_local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    // create position vectors
    Vector3f pos_takeoff       (0.0f,  0.0f, 2.0f);
    Vector3f pos_drone         (0.0f,  0.0f, 0.0f);
    Vector3f pos_current_goal;

    // fills them
    pos_current_goal = pos_takeoff;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos_current_goal));
        ros::spinOnce();
        rate.sleep();
    }


    // to arm drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // time storage
    ros::Time time_last_request = ros::Time::now();
    ros::Time time_start_hover = ros::Time::now();
    ros::Time time_last_print   = ros::Time::now();


    // parameters
    float precision = 0.5f;     // precision to reach the waypoints
    std::string answer;         // string returned by the server when sending position
    int state = 0;              // FSM state
    float hover_time = 10.0f;   // default hovering time at measure positions

    // state booleans
    bool bool_wait_for_offboard = true;
    bool bool_stop_all = false;

    
    // while ROS is online
    while(ros::ok()){

        // stop bool
        if(bool_stop_all){
            ROS_INFO("Stop condition reached");
            break;
        }

        // display info
        /*if( (ros::Time::now() - time_last_print) > ros::Duration(1.0) ){
            char state_char[current_state.mode.size()+1];
            strcpy(state_char, current_state.mode.c_str());
            ROS_INFO("Current state: %s, drone armed: %d", state_char, current_state.armed); 
            ROS_INFO("Current position: x=%.2f, y=%.2f, z=%.2f", pos_drone(0), pos_drone(1), pos_drone(2));
            ROS_INFO("Current goal:  x=%.2f, y=%.2f, z=%.2f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
            ROS_INFO("");
            time_last_print = ros::Time::now();
        }*/

        // waiting for OFFBOARD mode
        if(bool_wait_for_offboard){

            // every 1s:
            if((ros::Time::now() - time_last_request) > ros::Duration(1.0) ){

                // check state
                if(current_state.mode == "OFFBOARD"){
                    ROS_INFO("Offboard mode set");
                    bool_wait_for_offboard = false;
                    time_last_request = ros::Time::now();
                }
                else{
                    ROS_INFO("Waiting for offboard mode");
                    time_last_request = ros::Time::now();
                }
            }
        }
        else{
            // mode was set on OFFBOARD at one point

            // every 2s, try to arm drone
            if(!current_state.armed && (ros::Time::now() - time_last_request > ros::Duration(2.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    answer = send_GPS(pos_drone, ros::Time::now().toSec(), (char*)"drone_armed");
                    
                    // check if server online
                    char answer_char[answer.size()+1];
                    strcpy(answer_char, answer.c_str());
                    if(answer_char[0]=='4' && answer_char[1]=='0' && answer_char[2]=='4') {
                        ROS_INFO("ERROR: server is offline");
                        bool_stop_all = true;
                    }

                    // get next waypoint
                    pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);        
                }
                else{
                    ROS_INFO("Trying to arm drone");
                }

                time_last_request = ros::Time::now();
            }
            
            if(current_state.armed){
                // drone is armed and ready to fly

                // get new position
                ros::spinOnce();
                rate.sleep();
                pos_drone = conversion_to_vect(est_local_pos);

                // FSM
                switch(state){
                    case 0:{
                        // go to takeoff position
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Takeoff position reached!");
                            answer = send_GPS(pos_drone, ros::Time::now().toSec(), (char*)"drone_takeoff");
                            pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);
                            state = 1;
                        }
                        break;
                    }

                    case 1:{
                        // go to takeoff position
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Waypoint reached!");
                            answer = send_GPS(pos_drone, ros::Time::now().toSec(), (char*)"waypoint_reached");
                            hover_time = parse_hover_time_from_answer(answer);
                            state = 2;
                            time_start_hover = ros::Time::now();
                        }
                        break;
                    }

                    case 2:{
                        // collect data every second
                        if(ros::Time::now() - time_last_request > ros::Duration(1.0)){
                            ROS_INFO("Sending current position"); //, ts is %f", ros::Time::now().toSec());
                            answer = send_GPS(pos_drone, ros::Time::now().toSec(), (char*)"data_collected");
                            time_last_request = ros::Time::now();
                            
                            // time hovering is passed
                            if(ros::Time::now() - time_start_hover > ros::Duration(hover_time)){
                                ROS_INFO("Time spend hovering is over, next waypoint");
                                pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);
                                
                                char answer_char[answer.size()+1];
                                strcpy(answer_char, answer.c_str());
                                if(answer_char[0]=='N')          // new waypoint
                                    state = 1;
                                else if(answer_char[0]=='L')     // go to landing
                                    state = 3;
                            }                            
                        }
                        break;
                    }

                    case 3:{
                        // landing state
                        if((pos_drone-pos_current_goal).norm()<precision){
                            arm_cmd.request.value = false;
                            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                                ROS_INFO("Drone landing spot reached!");
                                send_GPS(pos_drone, ros::Time::now().toSec(), (char*)"drone_landing");
                                state = 4;
                            }
                        }
                        break;
                    }

                    case 4:{
                        // finished
                        ROS_INFO("Drone landed");
                        bool_stop_all = true;
                        break;
                    }


                    default:{
                        ROS_INFO("Unknown state");
                        break;
                    }
                }
            }
        }

        // ROS update
        local_pos_pub.publish(conversion_to_msg(pos_current_goal));
        ros::spinOnce();
        rate.sleep();
        pos_drone = conversion_to_vect(est_local_pos);
    }

    return 0;
}



// returns the waypoint based on the received string
Vector3f parse_WP_from_answer(std::string answer_string, Vector3f pos_current_goal){

    // string to char*
    char answer[answer_string.size()+1];
    strcpy(answer, answer_string.c_str());

    if(answer[0]=='N' || answer[0]=='L'){
        // find positions from message
        float pos_x, pos_y, pos_z;
        for (int i = 0; i < answer_string.size(); ++i){
            if(answer[i]=='x'){
                pos_x = atoi(answer+i+1);
            }
            if(answer[i]=='y'){
                pos_y = atoi(answer+i+1);
            }
            if(answer[i]=='z'){
                pos_z = atoi(answer+i+1);
            }
        }
        // create next waypoint
        Vector3f pos_new_goal (pos_x, pos_y, pos_z);

        return pos_new_goal;
    }

    // default, return current goal
    printf("ERROR: could not find new goal, returning pos_current_goal\n");
    return pos_current_goal;
}

// get the hoevering time from received string
float parse_hover_time_from_answer(std::string answer_string){

    // string to char*
    char answer[answer_string.size()+1];
    strcpy(answer, answer_string.c_str());

    // find hover time from message
    if(answer[0]=='H'){
        float hover_time;
        for (int i = 0; i < answer_string.size(); ++i)
        {
            if(answer[i]=='h'){
                hover_time = atoi(answer+i+1);
                return hover_time;
            }
        }
    }

    // default if nothing was read
    printf("ERROR: could not get hover time, returning 0.0f\n");
    return 0.0f;
}


// define the callbacks for the received ROS messages
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos){
    est_local_pos = *est_pos;
}


// convertion functions between Vector3f and PoseStamped ROS message
geometry_msgs::PoseStamped conversion_to_msg(Vector3f vect){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = vect(0);
    msg.pose.position.y = vect(1);
    msg.pose.position.z = vect(2);
    return msg;
}
Vector3f conversion_to_vect(geometry_msgs::PoseStamped msg){
    Vector3f vect;
    vect(0) = msg.pose.position.x;
    vect(1) = msg.pose.position.y;
    vect(2) = msg.pose.position.z;
    return vect;
}