/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>

// define the MAVROS messages
mavros_msgs::State current_state;               // drone state
geometry_msgs::PoseStamped  est_local_pos;      // local pose

// define the callbacks
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos){
    est_local_pos = *est_pos;
}

// To convert a vector into msg format
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = a(0);
    msg.pose.position.y = a(1);
    msg.pose.position.z = a(2);
    return msg;
}

// To convert a msg format in vector
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a){
    Vector3f v;
    v(0) = a.pose.position.x;
    v(1) = a.pose.position.y;
    v(2) = a.pose.position.z;
    return v;
}





// main
int main(int argc, char **argv)
{
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

    // create the vectors
    Vector3f pos_takeoff (0.0f,  0.0f, 2.0f);
    Vector3f pos_drone   (0.0f,  0.0f, 0.0f);
    Vector3f pos_goal = pos_takeoff;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos_takeoff));
        ros::spinOnce();
        rate.sleep();
    }

    // to go into OFFBOARD mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // to arm drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    
    // while ROS is online
    while(ros::ok()){

        // every 5s, try to set mode as OFFBOARD
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){

            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();

        } else {
            // drone current mode is OFFBOARD

            // every 5s, try to arm drone
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
            if (current_state.armed){
                // drone is armed and ready to fly

                // get new position
                ros::spinOnce();
                rate.sleep();
                pos_drone = conversion_to_vect(est_local_pos);
                ROS_INFO("Position: x=%.1f, y=%.1f, z=%.1f]", pos_drone(0), pos_drone(1), pos_drone(2));

                // if waypoint cleared, next one
                if ((pos_drone-pos_goal).norm()<0.1f){
                    Vector3f pos_increment (0.0f, 1.0f, 0.0f);
                    pos_goal = pos_goal + pos_increment;
                }

                // set new position at 1m in front
                local_pos_pub.publish(conversion_to_msg(pos_goal));
            }

        }

        local_pos_pub.publish(conversion_to_msg(pos_goal));
        ros::spinOnce();
        rate.sleep();
        pos_drone = conversion_to_vect(est_local_pos);
    }

    return 0;
}

