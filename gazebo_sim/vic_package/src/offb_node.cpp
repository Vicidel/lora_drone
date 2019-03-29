/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

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
#include "tcp_client.h"


// define the received MAVROS messages
mavros_msgs::State current_state;               // drone state (for OFFBOARD mode)
geometry_msgs::PoseStamped  est_local_pos;      // local position (x, y, z)


// functions definitions
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos);
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
void create_triangle_waypoints(Vector3f pos_center, float radius, Vector3f *waypoints_array);



// main
int main(int argc, char **argv)
{    
    printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    // ROS initialization
    ros::init(argc, argv, "offb_node");
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
    Vector3f pos_center_circle (0.0f,  0.0f, 15.0f);
    Vector3f waypoints_array[3];
    Vector3f pos_current_goal;

    // fills them
    pos_current_goal = pos_takeoff;
    create_triangle_waypoints(pos_center_circle, 100.0f, waypoints_array);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos_takeoff));
        ros::spinOnce();
        rate.sleep();
    }

    // to set OFFBOARD mode and to arm drone
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // time of last request (to avoid overloading)
    ros::Time last_request = ros::Time::now();


    // parameters
    int state = 0;
    float precision = 0.2f;


    // testing
    send_tcp();


    // while ROS is online
    while(ros::ok()){

        // every 5s, try to set mode as OFFBOARD
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){

            if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();

        } else {
            // drone current mode is OFFBOARD

            // every 5s, try to arm drone
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
            if(current_state.armed){
                // drone is armed and ready to fly

                // get new position
                ros::spinOnce();
                rate.sleep();
                pos_drone = conversion_to_vect(est_local_pos);
                //ROS_INFO("Position: x=%.1f, y=%.1f, z=%.1f", pos_drone(0), pos_drone(1), pos_drone(2));

                // FSM
                switch(state){
                    case 0:{
                        pos_current_goal = waypoints_array[0];
                        ROS_INFO("New goal: x=%.1f, y=%.1f, z=%.1f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
                        state = 1;
                        break;
                    }

                    case 1:{
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Goal reached!");
                            state = 2;
                        }
                        break;
                    }

                    case 2:{
                        pos_current_goal = waypoints_array[1];
                        ROS_INFO("New goal: x=%.1f, y=%.1f, z=%.1f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
                        state = 3;
                        break;
                    }

                    case 3:{
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Goal reached!");
                            state = 4;
                        }
                        break;
                    }

                    case 4:{
                        pos_current_goal = waypoints_array[2];
                        ROS_INFO("New goal: x=%.1f, y=%.1f, z=%.1f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
                        state = 5;
                        break;
                    }

                    case 5:{
                        if((pos_drone-pos_current_goal).norm()<precision){
                            ROS_INFO("Goal reached!");
                            state = 6;
                        }
                        break;
                    }

                    case 6:{
                        pos_current_goal = pos_drone;
                        ROS_INFO("Hovering in place");
                        break;
                    }

                    default:{
                        ROS_INFO("Unknown state");
                        break;
                    }
                }

                // set new position at 1m in front
                local_pos_pub.publish(conversion_to_msg(pos_current_goal));
            }

        }

        local_pos_pub.publish(conversion_to_msg(pos_current_goal));
        ros::spinOnce();
        rate.sleep();
        pos_drone = conversion_to_vect(est_local_pos);
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

// creates three waypoints in a triangle shape
void create_triangle_waypoints(Vector3f pos_center, float radius, Vector3f *waypoints_array){

    // first waypoint, S
    waypoints_array[0] << pos_center(0), pos_center(1) - radius, pos_center(2);

    // second waypoint, NE
    waypoints_array[1] << pos_center(0) + radius*sqrt(3)/2, pos_center(1) + radius/2, pos_center(2);

    // third waypoint, NW
    waypoints_array[2] << pos_center(0) - radius*sqrt(3)/2, pos_center(1) + radius/2, pos_center(2);
}