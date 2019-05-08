/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

// standard libraries
#include <stdio.h>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// libraries for Vector3f
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation

// ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>

// for server communication
#include "server_app.h"


// define the received MAVROS messages
mavros_msgs::State current_state;               // drone state (for OFFBOARD mode)
mavros_msgs::HomePosition home_position;        // home position 
geometry_msgs::PoseStamped est_local_pos;       // local position


// ROS callbacks
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg);

// conversion functions
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
mavros_msgs::PositionTarget conversion_to_target(Vector3f current, Vector3f goal);

// parse POST answer
Vector3f parse_WP_from_answer(std::string answer_string, Vector3f pos_current_goal);
float parse_hover_time_from_answer(std::string answer_string);

// function for threading
float send_firebase(bool bool_wait_for_offboard, ros::Time time_last_send_firebase, float time_firebase_period, mavros_msgs::HomePosition home_position, Vector3f pos_drone);


// main
int main(int argc, char **argv){    
    //printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    // ROS initialization
    ros::init(argc, argv, "vic_mission");
    ros::NodeHandle nh;

    // subscribes to topics 
    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    ros::Subscriber est_local_pos_sub   = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, est_local_pos_cb);
    ros::Subscriber home_pos_sub        = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, home_pos_cb);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    ros::Publisher target_pub           = nh.advertise<mavros_msgs::PositionTarget> ("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Waiting for FCU connection");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection done!");

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


    // to arm drone and set OFFBOARD
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // time storage
    ros::Time time_last_request       = ros::Time::now();
    ros::Time time_start_hover        = ros::Time::now();
    ros::Time time_last_print         = ros::Time::now();
    ros::Time time_last_send_firebase = ros::Time::now();
    ros::Time time_last_server_check  = ros::Time::now();

    // time parameters
    float time_firebase_period        = 0.5f;  // period of sending messages to Firebase
    float time_offboard_arm_period    = 4.0f;  // period to check offboard and arming
    float time_server_check_period    = 4.0f;  // period for kill switch and server offboard

    // parameters
    float precision = 0.5f;     // precision to reach the waypoints
    std::string answer;         // string returned by the server when sending position
    int state = 0;              // FSM state
    float hover_time = 10.0f;   // default hovering time at measure positions
    bool bool_fly_straight = true;      // fly in direction of waypoint or just x+
    int server_answer;
    int no_drone = 3;
    
    // state booleans
    bool bool_wait_for_offboard = true;
    bool bool_stop_all = false;

    
    // while ROS is online
    while(ros::ok()){

        // check from server
        if(ros::Time::now() - time_last_server_check > ros::Duration(time_server_check_period)) {
            server_answer = check_server(no_drone);    // 0:nothing, 1:go into offboard, 666:kill
            ROS_INFO("Answer from server check: %d", server_answer);
            if(server_answer==666) {
                // disarm drone and stop program
                arm_cmd.request.value = false;
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    bool_stop_all = true;
                    ROS_INFO("Server kill switch activated!");
                    break;
                }
            }
            time_last_server_check = ros::Time::now();
        }

        // stop bool
        if(bool_stop_all){
            ROS_INFO("Stop condition reached");
            break;
        }

        // send to Firebase in a new thread
        std::thread firebase_thread(send_firebase, bool_wait_for_offboard, time_last_send_firebase, time_firebase_period, home_position, pos_drone);

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
            if((ros::Time::now() - time_last_request) > ros::Duration(time_offboard_arm_period) ){

                // check from server
                if(server_answer==1){
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled from server");
                        bool_wait_for_offboard = false;
                    }
                }
                // check from RC
                else if(current_state.mode == "OFFBOARD"){
                    ROS_INFO("Offboard enabled from RC");
                    bool_wait_for_offboard = false;
                }
                // still waiting
                else{
                    ROS_INFO("Waiting for offboard mode (from server or RC)");
                }

                time_last_request = ros::Time::now();
            }
        }
        else{
            // mode was set on OFFBOARD at one point

            // check if we got back to manual mode
            if(current_state.mode == "MANUAL"){
                ROS_INFO("Switched back to manual mode");
                bool_wait_for_offboard = true;
            }

            // every 2s, try to arm drone
            if(!current_state.armed && (ros::Time::now() - time_last_request > ros::Duration(time_offboard_arm_period))) {
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
                    else{
                        // get next waypoint
                        pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);
                    }
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
                                bool_stop_all = true;
                            }
                        }
                        break;
                    }

                    default:{
                        ROS_INFO("Unknown state");
                        break;
                    }
                }
            }
        }

        // rejoin threads
        firebase_thread.join();
        
        // ROS update
        if(bool_fly_straight) target_pub.publish(conversion_to_target(pos_drone, pos_current_goal));
        else local_pos_pub.publish(conversion_to_msg(pos_current_goal));
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
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    est_local_pos = *msg;
}
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    home_position = *msg;
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
mavros_msgs::PositionTarget conversion_to_target(Vector3f current, Vector3f goal){
    // create message and bit_mask
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // set position target
    msg.position.x = goal(0);
    msg.position.y = goal(1);
    msg.position.z = goal(2);

    // set yaw target
    double yaw_target = atan((goal(1)-current(1))/(goal(0)-current(0)));
    if(goal(0)<current(0)) yaw_target = yaw_target + 3.1414592;
    //ROS_INFO("goal x %f y %f  curretn x %f y %f", goal(0), goal(1), current(0), current(1));
    //ROS_INFO("yaw degree computed is %f (rad is %f)", yaw_target*180/3.141, yaw_target);
    msg.yaw = yaw_target;
    return msg;
}


// function for threading
float send_firebase(bool bool_wait_for_offboard, ros::Time time_last_send_firebase, float time_firebase_period, mavros_msgs::HomePosition home_position, Vector3f pos_drone){
    if(ros::Time::now() - time_last_send_firebase > ros::Duration(time_firebase_period)) {
        if(bool_wait_for_offboard) send_home_firebase(home_position.geo.latitude, home_position.geo.longitude, home_position.geo.altitude, home_position.position.x, home_position.position.y, home_position.position.z, ros::Time::now().toSec());
        send_GPS_firebase(pos_drone, ros::Time::now().toSec());
        time_last_send_firebase = ros::Time::now();
    }
}