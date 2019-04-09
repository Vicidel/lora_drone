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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>

// for server communication
#include "server_app.h"


// define the received MAVROS messages
mavros_msgs::State current_state;               // drone state (for OFFBOARD mode)
geometry_msgs::PoseStamped  est_local_pos;      // local position (x, y, z)
geometry_msgs::PoseWithCovarianceStamped  est_global_pos;      // global position (x, y, z)


// functions definitions
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos);
void est_global_pos_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& est_pos_gps);
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
Vector3f conversion_to_vect_v2(geometry_msgs::PoseWithCovarianceStamped msg);


// main
int main(int argc, char **argv){    
    //printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    // ROS initialization
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // subscribes to topics 
    ros::Subscriber state_sub          = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    ros::Subscriber est_local_pos_sub  = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, est_local_pos_cb);
    ros::Subscriber est_global_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("mavros/global_position/local", 10, est_global_pos_cb);
    ros::Publisher local_pos_pub       = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
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
    Vector3f pos_drone_gps     (0.0f,  0.0f, 0.0f);
    Vector3f pos_home          (0.0f,  0.0f, 0.0f);
    Vector3f pos_current_goal;

    // fills them
    pos_current_goal = pos_takeoff;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos_current_goal));
        ros::spinOnce();
        rate.sleep();
    }

    // to set OFFBOARD mode and to arm drone
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // time storage
    ros::Time time_last_request = ros::Time::now();
    ros::Time time_start_hover  = ros::Time::now();
    ros::Time time_last_print   = ros::Time::now();


    // parameters
    float precision = 0.1f;     // precision to reach the waypoints
    int state = 0;              // FSM state
    float hover_time = 20.0f;   // hovering time at measure positions
    bool drone_doing_stuff = true;

    
    // while ROS is online
    while(ros::ok() && drone_doing_stuff){

        // display info
        if( (ros::Time::now() - time_last_print) > ros::Duration(1.0) ){
            char state_char[current_state.mode.size()+1];
            strcpy(state_char, current_state.mode.c_str());
            ROS_INFO("Current state: %s, drone armed: %d", state_char, current_state.armed); 
            ROS_INFO("Current position: x=%.2f, y=%.2f, z=%.2f", pos_drone(0), pos_drone(1), pos_drone(2));
            ROS_INFO("Current position (gps): x=%.2f, y=%.2f, z=%.2f", pos_drone_gps(0), pos_drone_gps(1), pos_drone_gps(2));
            ROS_INFO("Current goal:  x=%.2f, y=%.2f, z=%.2f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
            ROS_INFO("");
            time_last_print = ros::Time::now();
        }

        // every 5s, try to set mode as OFFBOARD if switch is in MANUAL
        if(current_state.mode == "MANUAL"){
            if( (ros::Time::now() - time_last_request) > ros::Duration(2.0) ){

                // this block for setting OFFBOARD from script
                //if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                //    ROS_INFO("Offboard enabled from script");
                //    pos_home = pos_drone;
                //}

                // this block to wait for OFFBOARD from RC
                ROS_INFO("Waiting for offboard mode from RC, keeping drone disarmed");
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);

                time_last_request = ros::Time::now();
            }

        } else{
            // drone current mode is OFFBOARD (or at least not MANUAL)

            // every 5s, try to arm drone
            if(!current_state.armed && (ros::Time::now() - time_last_request > ros::Duration(2.0))){
                arm_cmd.request.value = true;
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

                // get arming position
                ros::spinOnce();
                rate.sleep();
                pos_home = conversion_to_vect(est_local_pos);
                pos_drone_gps = conversion_to_vect_v2(est_global_pos);

                // set goal as 1m higher
                pos_current_goal = pos_home;
                pos_current_goal(2) = pos_current_goal(2) + 1.0f;

                time_last_request = ros::Time::now();
            }
            
            if(current_state.armed){
                // drone is armed and ready to fly

                // get new position
                ros::spinOnce();
                rate.sleep();
                pos_drone = conversion_to_vect(est_local_pos);
                pos_drone_gps = conversion_to_vect_v2(est_global_pos);
                
                // FSM
                switch(state){
                    case 0:{
                        // go to takeoff position
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Takeoff position reached!");
                            time_start_hover = ros::Time::now();
                            state = 1;
                        }
                        break;
                    }

                    case 1:{
                        // hovering 
                        ROS_INFO("Hovering at takeoff position");

                        // time hovering is passed
                        if(ros::Time::now() - time_start_hover > ros::Duration(hover_time)){
                            ROS_INFO("Time spend hovering is over, landing");
                            pos_current_goal = pos_home;
                            state = 2;
                        }
                        
                        break;
                    }

                    case 2:{
                        // landing state
                        if((pos_drone-pos_current_goal).norm()<precision){
                            arm_cmd.request.value = false;
                            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                                ROS_INFO("Drone landing spot reached!");
                                state = 3;
                            }
                        }
                        break;
                    }

                    case 3:{
                        // finished
                        ROS_INFO("Drone landed");
                        drone_doing_stuff = false;
                        break;
                    }

                    default:{
                        ROS_INFO("Unknown state");
                        break;
                    }
                }
            }

        }

        local_pos_pub.publish(conversion_to_msg(pos_current_goal));
        ros::spinOnce();
        rate.sleep();
        pos_drone = conversion_to_vect(est_local_pos);
        pos_drone_gps = conversion_to_vect_v2(est_global_pos);
    }

    return 0;
}



// define the callbacks for the received ROS messages
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos){
    est_local_pos = *est_pos;
}
void est_global_pos_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& est_pos_gps){
    est_global_pos = *est_pos_gps;
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
    ROS_INFO("");
    ROS_INFO("new pose: %f %f %f", vect(0), vect(1), vect(2));

    return vect;
}
Vector3f conversion_to_vect_v2(geometry_msgs::PoseWithCovarianceStamped msg){
    Vector3f vect;
    vect(0) = msg.pose.pose.position.x;
    vect(1) = msg.pose.pose.position.y;
    vect(2) = msg.pose.pose.position.z;
    ROS_INFO("new gps pose: %f %f %f", vect(0), vect(1), vect(2));
    ROS_INFO("");

    return vect;
}