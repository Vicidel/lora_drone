/*
File: vic_mission_v2.cpp
Author: Victor Delafontaine
Date: May 2019

The drone receives all commands from the Swisscom server (needs to be online). 
They can be hovering time, new waypoints, landing waypoints...
Uses the server_app files for communication with server
*/


/**************************************************************************
******************************   INCLUDE   ********************************
***************************************************************************/

// standard libraries
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// for threading
#include <thread> 
#include <future>
#include <chrono>

// libraries for Vector3f
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation

// ROS main library and messages 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HilGPS.h>

// for server communication
#include "server_app.h"



/**************************************************************************
**************************   GLOBAL VARIABLES   ***************************
***************************************************************************/

// define the received MAVROS messages
mavros_msgs::State current_state;               // drone state (for OFFBOARD mode)
mavros_msgs::HomePosition home_position;        // home position 
geometry_msgs::PoseStamped est_local_pos;       // local position (xyz)
sensor_msgs::NavSatFix est_global_pos;          // global position (GPS)



/**************************************************************************
************************   FUNCTION DECLARATION   *************************
***************************************************************************/

// ROS callbacks
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg);
void est_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);

// conversion functions
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a, geometry_msgs::PoseStamped est_local_pos);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
mavros_msgs::PositionTarget conversion_to_target(Vector3f current, Vector3f goal);

// parse POST answer
Vector3f parse_WP_from_answer(std::string answer_string, Vector3f pos_current_goal);
float parse_hover_time_from_answer(std::string answer_string);

// for sending home and drone position to Firebase
ros::Time send_firebase(bool bool_wait_for_offboard, ros::Time time_last_send_firebase, 
    float time_firebase_period, mavros_msgs::HomePosition home_position, 
    sensor_msgs::NavSatFix est_global_pos, int drone_id, std::string state, double relative_altitude, int fsm_state);

// check if server is OK by its answer
bool check_server_answer(std::string answer);

// for threading
std::string threaded_send_drone_state(Vector3f position, sensor_msgs::NavSatFix est_global_pos, double time, char* payload, 
    int drone_id, int nb_drone, bool bool_no_answer, ros::Rate rate, bool bool_fly_straight, 
    Vector3f pos_drone, Vector3f pos_current_goal, ros::Publisher target_pub, ros::Publisher local_pos_pub);
void threaded_ros_update(ros::Rate rate, bool bool_fly_straight, Vector3f pos_drone, Vector3f pos_current_goal, 
    ros::Publisher target_pub, ros::Publisher local_pos_pub, std::future<void> future_object);

// ask user for input
void user_input_sim_param(bool& bool_continuous_sim, int& nb_drone, int& drone_id);


/***************************************************************************
***************************   MAIN FUNCTION   *****************************
***************************************************************************/

int main(int argc, char **argv){    
    //printf("In %s file, %s function\n", __FILE__, __FUNCTION__);


    /**************************************************************************
    ***************************  INITIALIZATION   *****************************
    ***************************************************************************/

    // ROS initialization
    ros::init(argc, argv, "vic_mission");
    ros::NodeHandle nh;

    // ROS topic subscriptions ("get information")
    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
    ros::Subscriber est_local_pos_sub   = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, est_local_pos_cb);
    ros::Subscriber home_pos_sub        = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, home_pos_cb);
    ros::Subscriber global_pos_sub      = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global", 10, est_global_pos_cb);

    // ROS services ("do something")
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    ros::ServiceClient takeoff_client   = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/takeoff");
    ros::ServiceClient landing_client   = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land");

    // ROS publishers ("post information")
    ros::Publisher target_pub           = nh.advertise<mavros_msgs::PositionTarget> ("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    
    // ROS publishing rate (MUST be faster than 2Hz)
    ros::Rate rate(20.0);


    // waiting for FCU connection with drone
    ROS_INFO("Waiting for FCU connection");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection done!");


    /**************************************************************************
    ************************  SIMULATION PARAMETERS   *************************
    ***************************************************************************/

    // parameters to change depending on simulation type
    int drone_id;           // can be 1, 2 or 3
    int nb_drone;           // can be 1 or 3
    bool bool_continuous_sim;    // continuous (true) or classic (false) sim type

    // call user input function
    user_input_sim_param(bool_continuous_sim, nb_drone, drone_id);


    /**************************************************************************
    ******************************  VARIABLES   *******************************
    ***************************************************************************/

    // create position vectors
    Vector3f pos_drone         (0.0f,  0.0f, 0.0f);
    Vector3f pos_current_goal = pos_drone;

    // variable for ROS services to arm drone and set OFFBOARD mode
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    // time storage
    ros::Time time_last_request       = ros::Time::now();
    ros::Time time_start_hover        = ros::Time::now();
    ros::Time time_last_print         = ros::Time::now();
    ros::Time time_last_send_firebase = ros::Time::now();
    ros::Time time_last_server_check  = ros::Time::now();

    // time parameters
    float time_firebase_period        = 0.5f;     // period of sending messages to Firebase
    float time_offboard_arm_period    = 4.0f;     // period to check RC offboard and arming
    float time_kill_check_period      = 1.0f;     // period for server kill switch
    float time_data_collection_period = 1.0f;     // period for data collection when hovering
    float time_data_collection_temp_period = 2.0f;     // period for data collection when moving (temp)

    // misc variables
    float precision = 1.5f;              // precision to reach
    std::string answer;                  // string returned by the server when sending position
    int state = 0;                       // FSM state
    float hover_time;                    // hovering time at measure positions
    bool bool_fly_straight = false;      // fly in direction of waypoint or just keep same yaw
    int gmaps_safety_switch = 0;         // 0 for unkill, 1 for kil, 2 for hover, 3 for RTL

    // state booleans
    bool bool_wait_for_offboard = true;  // waiting for offboard mode activation from RC or server app
    bool bool_stop_all = false;          // stop all when simulation is ended or kill switch
    bool bool_pause_all = false;         // when server has problem, start hovering
    bool bool_ignore = false;            // activated when hover or RTL switch active
    
    // set precision based on sim type
    if(bool_continuous_sim) precision = 2.0f;
    else precision = 1.0f;


    /**************************************************************************
    ***************************  START ROS LOOP   *****************************
    ***************************************************************************/

    // empty Firebase
    ROS_INFO("Emptying Firebase for GMaps GUI");
    empty_firebase(drone_id);

    // send a few setpoints before starting
    ROS_INFO("Sending a few waypoints before start");
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos_current_goal, est_local_pos));
        ros::spinOnce();
        pos_drone = conversion_to_vect(est_local_pos);
        rate.sleep();
    }

    // while ROS is online
    ROS_INFO("Starting the ROS loop");
    while(ros::ok()){


        /**************************************************************************
        ***************************  SAFETY FEATURES   ****************************
        ***************************************************************************/

        /***********************   CHECK GMAPS SWITCHES   ************************/
        if(ros::Time::now() - time_last_server_check > ros::Duration(time_kill_check_period)) {
            // check offboard mode and kill switch from GMaps API through server
            gmaps_safety_switch = check_kill_server();
            if(gmaps_safety_switch==2){
                ROS_WARN("GUI hovering switch activated");
            }
            else if(gmaps_safety_switch==3){
                ROS_WARN("GUI RTL switch activated");
            }
            
            // store current time
            time_last_server_check = ros::Time::now();
        }

        /*************************   GMAPS KILL   **************************/
        if(gmaps_safety_switch==1){
            // disarm drone and stop program
            arm_cmd.request.value = false;
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_WARN("GUI kill switch activated!");
                break;
            }
        }

        /*************************   GMAPS HOLD   **************************/
        else if(gmaps_safety_switch==2){
            // start hovering in position
            set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                bool_ignore = true;
                bool_wait_for_offboard = true;
            }
        }

        /*************************   GMAPS RTL   **************************/
        else if(gmaps_safety_switch==3){
            // return to landing
            set_mode.request.custom_mode = "AUTO.RTL";
            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                bool_ignore = true;
                bool_wait_for_offboard = true;
            }
        }

        /**********************   NO GMAPS SWITCH   ***********************/
        else{
            bool_ignore = false;
        }

        /*************************   ROS KILL   **************************/
        if(bool_stop_all){
            // set at manual, disarm drone and stop program
            set_mode.request.custom_mode = "MANUAL";
            arm_cmd.request.value = false;
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                    ROS_WARN("Stop condition reached");
                    break;
                }
            }
        }

        /*************************   ROS PAUSE   **************************/
        if(bool_pause_all){
            // start hovering in position
            set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                bool_wait_for_offboard = true;
            }
        }


        /**************************************************************************
        ***************************  DEBUGGING PRINT   ****************************
        ***************************************************************************/

        /*if( (ros::Time::now() - time_last_print) > ros::Duration(1.0) ){
            char state_char[current_state.mode.size()+1];
            strcpy(state_char, current_state.mode.c_str());
            ROS_INFO("Current state: %s, drone armed: %d", state_char, current_state.armed); 
            ROS_INFO("Current position: x=%.2f, y=%.2f, z=%.2f", pos_drone(0), pos_drone(1), pos_drone(2));
            ROS_INFO("Current position GPS: lat=%.2f, lng=%.2f", est_global_pos.latitude, est_global_pos.longitude);
            ROS_INFO("Current goal:  x=%.2f, y=%.2f, z=%.2f", pos_current_goal(0), pos_current_goal(1), pos_current_goal(2));
            ROS_INFO("");
            time_last_print = ros::Time::now();
        }*/


        /**************************************************************************
        ******************************    MAIN CODE    ****************************
        ***************************************************************************/

        // ignore all the loop, for when safety switch in place
        if(!bool_ignore){

            
            /**************************************************************************
            **************************   OFFBOARD CHECKING   **************************
            ***************************************************************************/
            if(bool_wait_for_offboard){
                if((ros::Time::now() - time_last_request) > ros::Duration(time_offboard_arm_period) ){

                    /*************************   FROM SERVER   **************************/
                    if(check_offboard_server(drone_id)){

                        // set mode as offboard
                        set_mode.request.custom_mode = "OFFBOARD";
                        if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){

                            // post on server
                            ROS_INFO("Offboard enabled from server");
                            answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"offb", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);

                            // check server error
                            if(check_server_answer(answer))
                                bool_pause_all = true;
                            else{
                                // next state: drone arming
                                bool_wait_for_offboard = false;
                                pos_current_goal = pos_drone;

                                // if drone retook off after partial mission: redo last FSM state
                                if(state!=0 && state!=666) state = state - 1;
                            }
                        }
                    }
                    
                    /*************************   FROM RC/QGC   **************************/
                    else if(current_state.mode == "OFFBOARD"){

                        // post on server
                        ROS_INFO("Offboard enabled from RC");
                        answer =threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"offb", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);

                        // check server error
                        if(check_server_answer(answer))
                            bool_pause_all = true;
                        else{
                            // next state: drone arming
                            bool_wait_for_offboard = false;
                            pos_current_goal = pos_drone;

                            // if drone retook off after partial mission: redo last FSM state
                            if(state!=0 && state!=666) state = state - 1;
                        }
                    }

                    /***********************   STILL WAITING   ************************/
                    else{
                        ROS_INFO("Waiting for offboard mode (from server or RC)");
                    }

                    // store current time
                    time_last_request = ros::Time::now();
                }
            }

            /**************************************************************************
            **************************   DRONE IN MISSION   ***************************
            ***************************************************************************/
            else{

                // check if we got back to manual mode
                if(current_state.mode == "MANUAL"){
                    ROS_INFO("Switched back to manual mode");
                    bool_wait_for_offboard = true;
                }

                // check if we got into AUTO.LOITER, by safety feature (geofence) or pause (RC switch)
                if(current_state.mode == "AUTO.LOITER"){
                    ROS_INFO("Switched to automatic loiter mode (by safety or RC switch)");
                    bool_wait_for_offboard = true;
                }

                /**************************************************************************
                ****************************   DRONE ARMING   *****************************
                ***************************************************************************/
                if(!current_state.armed && (ros::Time::now() - time_last_request > ros::Duration(time_offboard_arm_period))) {

                    /***********************   SUCCESS   ************************/
                    if(arming_client.call(arm_cmd) && arm_cmd.response.success){

                        // post on server
                        ROS_INFO("Vehicle armed");
                        answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"arm", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                        
                        // check server error
                        if(check_server_answer(answer))
                            bool_pause_all = true;
                        else{
                            // next state: takeoff_altitude above current position
                            pos_current_goal = pos_drone;
                            pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);

                            // if drone retook off after partial mission: redo last FSM state
                            if(state!=0 && state!=666) state = state - 1;
                        }
                    }
                    /***********************   STILL TRYING   ************************/
                    else{
                        ROS_INFO("Trying to arm drone");
                    }

                    // store current time
                    time_last_request = ros::Time::now();
                }
                
                /**************************************************************************
                ****************************   DRONE IS ARMED   ***************************
                ***************************************************************************/
                if(current_state.armed){

                    // get new drone position
                    ros::spinOnce();
                    rate.sleep();
                    pos_drone = conversion_to_vect(est_local_pos);


                    /**************************************************************************
                    ****************************   MISSION FSM   ******************************
                    ***************************************************************************/
                    switch(state){

                        /**************************************************************************
                        ***************************   SHARED STATES   *****************************
                        ***************************************************************************/

                        /***********************   TAKING OFF   ************************/
                        case 0:{
                            if((pos_drone-pos_current_goal).norm()<precision){
                                // post on server
                                ROS_INFO("Takeoff position reached!");
                                answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"takeoff", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                
                                // check server error
                                if(check_server_answer(answer))
                                    bool_pause_all = true;
                                else{
                                    // next state: go to waypoint
                                    pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);
                                    if(bool_continuous_sim)
                                        state = 666;
                                    else
                                        state = 1;

                                    // drone flies in direction of waypoint for next state
                                    bool_fly_straight = true;
                                }
                            }
                            break;
                        }

                       /********************   GOING IN LANDING POSITION   *********************/
                        case 5:{
                            if((pos_drone-pos_current_goal).norm()<precision){
                                // reached position, start going down
                                ROS_INFO("Drone above landing spot, going down");
                                pos_current_goal(2) = pos_current_goal(2) - pos_drone(2);
                                state = 6;

                                // drone goes down keeping same orientation
                                bool_fly_straight = false;
                            }
                            break;
                        }

                        /***********************   GOING DOWN   ************************/
                        case 6:{
                            if((pos_drone-pos_current_goal).norm()<precision){
                                // reached ground level, disarm drone
                                arm_cmd.request.value = false;
                                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                                    ROS_INFO("Drone landing spot reached!");
                                    state = 7;
                                }
                            }
                            break;
                        }

                        /***********************   DISARMING   ************************/
                        case 7:{
                            ROS_INFO("Drone landed");
                            threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"land", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                            bool_stop_all = true;
                            break;
                        }

                        /********************   ?? SHOULDN'T HAPPEN ??   *******************/
                        default:{
                            ROS_INFO("Unknown state");
                            break;
                        }


                        /**************************************************************************
                        ***************************   CLASSIC STATES   ****************************
                        ***************************************************************************/

                        /***********************   GOING TO WAYPOINT   ************************/
                        case 1:{
                            // sending
                            if(ros::Time::now() - time_last_request > ros::Duration(time_data_collection_temp_period)){
                                threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"data", drone_id, nb_drone, true, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                time_last_request = ros::Time::now();
                            }

                            // reaached position
                            if((pos_drone-pos_current_goal).norm()<precision){
                                // position reached, sending position to drone_dataset and wait for next instruction
                                ROS_INFO("Waypoint reached!");
                                answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"wp_ok", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                
                                // check server error
                                if(check_server_answer(answer))
                                    bool_pause_all = true;
                                else{
                                    // next state: waiting for all drones to be ready
                                    state = 2;

                                    // drone waits while keeping same orientation
                                    bool_fly_straight = false;
                                }
                            }
                            break;
                        }

                        /********************   WAITING FOR OTHER DRONES   *********************/
                        case 2:{
                            if(ros::Time::now() - time_last_request > ros::Duration(time_data_collection_period)){
                                ROS_INFO("Waiting, sending current position"); //, ts is %f", ros::Time::now().toSec());
                                answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"wait", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                
                                // check server error
                                if(check_server_answer(answer))
                                    bool_pause_all = true;
                                else{
                                    // check if answer is start hover or wait again
                                    char answer_char[answer.size()+1];
                                    strcpy(answer_char, answer.c_str());
                                    if(answer_char[0]=='H'){          // hovering/collecting
                                        state = 3;
                                        time_start_hover = ros::Time::now();
                                        hover_time = parse_hover_time_from_answer(answer);
                                    }
                                    else if(answer_char[0]=='W'){     // still waiting
                                        state = 2;
                                        pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);
                                    }

                                    // store current time
                                    time_last_request = ros::Time::now();
                                }
                            }
                            break;
                        }

                        /******************   HOVERING AND COLLECTING DATA   *******************/
                        case 3:{
                            if(ros::Time::now() - time_last_request > ros::Duration(time_data_collection_period)){
                                // X seconds passed, sending posiion again
                                ROS_INFO("Sending current position");
                                answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"hover", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);

                                // check server error
                                if(check_server_answer(answer))
                                    bool_pause_all = true;
                                else{
                                    // time hovering is passed
                                    if(ros::Time::now() - time_start_hover > ros::Duration(hover_time)){
                                        ROS_INFO("Time spend hovering is over, next state");
                                        state = 4;
                                    }                            

                                    // store current time
                                    time_last_request = ros::Time::now();
                                }
                            }
                            break;
                        }

                        /********************   WAITING FOR NEXT WP   *********************/
                        case 4:{
                            ROS_INFO("Sending position waiting for the next waypoint");
                            answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"wait_next", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                            
                            // check server error
                            if(check_server_answer(answer))
                                bool_pause_all = true;
                            else{
                                pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);

                                // check if answer is next waypoint or landing
                                char answer_char[answer.size()+1];
                                strcpy(answer_char, answer.c_str());
                                if(answer_char[0]=='N')          // next waypoint
                                    state = 1;
                                else if(answer_char[0]=='L')     // landing waypoint
                                    state = 5;

                                // drone flies in direction of waypoint for next state
                                bool_fly_straight = true;
                            }
                            break;
                        }


                        /**************************************************************************
                        *************************   CONTINUOUS STATES   ***************************
                        ***************************************************************************/

                        /*******************   GOING TO WAYPOINT WHILE SENDING  ********************/
                        case 666:{

                            // sending
                            if(ros::Time::now() - time_last_request > ros::Duration(time_data_collection_temp_period)){
                                threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"data", drone_id, nb_drone, true, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                time_last_request = ros::Time::now();
                            }

                            // reached position
                            if((pos_drone-pos_current_goal).norm()<precision){
                                ROS_INFO("Waypoint reached!");
                                answer = threaded_send_drone_state(pos_drone, est_global_pos, ros::Time::now().toSec(), (char*)"wp_ok_cont", drone_id, nb_drone, false, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub);
                                
                                // check server error
                                if(check_server_answer(answer))
                                    bool_pause_all = true;
                                else{
                                    pos_current_goal = parse_WP_from_answer(answer, pos_current_goal);

                                    // check if answer is next waypoint or landing
                                    char answer_char[answer.size()+1];
                                    strcpy(answer_char, answer.c_str());
                                    if(answer_char[0]=='N')          // next waypoint
                                        state = 666;
                                    else if(answer_char[0]=='L')     // landing waypoint
                                        state = 5;
                                }
                            }
                            break;
                        }

                    }
                }
            }
        }


        /**************************************************************************
        ************************   ROS AND GMAPS UPDATE   *************************
        ***************************************************************************/
                    
        // ROS update
        if(bool_fly_straight) target_pub.publish(conversion_to_target(pos_drone, pos_current_goal));
        else local_pos_pub.publish(conversion_to_msg(pos_current_goal, est_local_pos));
        ros::spinOnce();
        pos_drone = conversion_to_vect(est_local_pos);
        rate.sleep();

        // sending to Firebase
        time_last_send_firebase = send_firebase(bool_wait_for_offboard, time_last_send_firebase, 
            time_firebase_period, home_position, est_global_pos, drone_id, current_state.mode, pos_drone(2), state);
    }

    // success
    return 0;
}



/**************************************************************************
**************************   OTHER FUNCTIONS   ****************************
***************************************************************************/

// returns the waypoint based on the received string
Vector3f parse_WP_from_answer(std::string answer_string, Vector3f pos_current_goal){

    // string to char* conversion
    char answer[answer_string.size()+1];
    strcpy(answer, answer_string.c_str());

    // takeoff altitude received
    if(answer[0]=='T'){
        float takeoff_altitude;
        for (int i = 0; i < answer_string.size(); ++i){
            if(answer[i]=='h'){
                takeoff_altitude = atoi(answer+i+1);
            }
        }
        // create next waypoint
        pos_current_goal(2) = pos_current_goal(2) + takeoff_altitude;

        return pos_current_goal;
    }

    // waiting, keep same goal
    if(answer[0]=='W'){
        return pos_current_goal;
    }


    // new waypoint received (can be new or landing)
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

    // hovering time received
    if(answer[0]=='H'){
        // find hover time from message
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
void est_global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    est_global_pos = *msg;
}


// convertion functions between Vector3f and PoseStamped ROS message
geometry_msgs::PoseStamped conversion_to_msg(Vector3f vect, geometry_msgs::PoseStamped est_local_pos){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = vect(0);
    msg.pose.position.y = vect(1);
    msg.pose.position.z = vect(2);
    msg.pose.orientation = est_local_pos.pose.orientation;
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


// function for sending to Firebase
ros::Time send_firebase(bool bool_wait_for_offboard, ros::Time time_last_send_firebase, 
        float time_firebase_period, mavros_msgs::HomePosition home_position, 
        sensor_msgs::NavSatFix est_global_pos, int drone_id, std::string state, double relative_altitude, int fsm_state){
    
    // every period
    if(ros::Time::now() - time_last_send_firebase > ros::Duration(time_firebase_period)) {
        
        // send home position
        if(bool_wait_for_offboard) send_home_firebase(home_position.geo.latitude, 
            home_position.geo.longitude, home_position.geo.altitude, 
            home_position.position.x, home_position.position.y, home_position.position.z, 
            ros::Time::now().toSec(), drone_id);

        // send drone position
        send_GPS_firebase(est_global_pos.latitude, est_global_pos.longitude, est_global_pos.altitude, 
            ros::Time::now().toSec(), drone_id, state, relative_altitude, fsm_state);

        // store current time
        time_last_send_firebase = ros::Time::now();
    }

    return time_last_send_firebase;
}


// check if server is OK by its answer
bool check_server_answer(std::string response_string){

    // cast in char* to check errors
    char answer_char[response_string.size()+1];
    strcpy(answer_char, response_string.c_str());

    // check 404
    if(answer_char[0]=='4' && answer_char[1]=='0' && answer_char[2]=='4') {
        ROS_WARN("ERROR: server is offline (404), pausing simulation");
        return true;
    }

    // check bug in server
    if(answer_char[0]=='<' && answer_char[1]=='!') {
        ROS_WARN("ERROR: server has an internal error, pausing simulation");
        return true;
    }

    // default 
    return false;
}


// send_drone_state but with threading imbedded
std::string threaded_send_drone_state(Vector3f position, sensor_msgs::NavSatFix est_global_pos, double time, char* payload, 
    int drone_id, int nb_drone, bool bool_no_answer, ros::Rate rate, bool bool_fly_straight, 
    Vector3f pos_drone, Vector3f pos_current_goal, ros::Publisher target_pub, ros::Publisher local_pos_pub){

    // create a std::promise object
    std::promise<void> exit_signal;
    std::future<void> future_object = exit_signal.get_future();

    // start ROS update thread (normal thread)
    std::thread thread_ROS(threaded_ros_update, rate, bool_fly_straight, pos_drone, pos_current_goal, target_pub, local_pos_pub, std::move(future_object));

    // start send_drone_state thread (async thread)
    auto thread_send = std::async(send_drone_state, position, est_global_pos.latitude, est_global_pos.longitude, est_global_pos.altitude, time, payload, drone_id, nb_drone, bool_no_answer);

    // finsihed send_state, send exit to ROS thread
    std::string answer = thread_send.get();
    exit_signal.set_value();
    thread_ROS.join();

    // return answer
    return answer;
}

// updates ROS while waiting for send_drone_state thread to finish
void threaded_ros_update(ros::Rate rate, bool bool_fly_straight, Vector3f pos_drone, Vector3f pos_current_goal, 
    ros::Publisher target_pub, ros::Publisher local_pos_pub, std::future<void> future_object){
    
    // while promise not set
    while(future_object.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout){
        
        // ROS update
        if(bool_fly_straight) target_pub.publish(conversion_to_target(pos_drone, pos_current_goal));
        else local_pos_pub.publish(conversion_to_msg(pos_current_goal, est_local_pos));
        ros::spinOnce();
        pos_drone = conversion_to_vect(est_local_pos);
        rate.sleep();
    }
}


// ask user for input
void user_input_sim_param(bool& bool_continuous_sim, int& nb_drone, int& drone_id){

    // user input for sim type
    std::cout << "Input simulation type (0=classic, 1=continuous): ";
    std::cin >> bool_continuous_sim;

    if(bool_continuous_sim){
        // user input
        std::cout << "Input number of drones for this simulation (1/3): ";
        std::cin >> nb_drone;
        if(nb_drone==3){
            std::cout << "Input drone id for this simulation (1/2/3): ";
            std::cin >> drone_id;
            ROS_WARN("Continuous simulation started with three drone (drone_id=%d)", drone_id);
        }
        else{
            drone_id = 1;       // drone R is default when using one drone
            ROS_WARN("Continuous simulation started with one drone (R)");
        }
    }
    else{
        // user input
        std::cout << "Input number of drones for this simulation (1/3): ";
        std::cin >> nb_drone;
        if(nb_drone==3){
            std::cout << "Input drone id for this simulation (1/2/3): ";
            std::cin >> drone_id;
            ROS_WARN("Classic simulation started with three drone (drone_id=%d)", drone_id);
        }
        else{
            drone_id = 1;       // drone R is default when using one drone
            ROS_WARN("Classic simulation started with one drone (R)");
        }
    }
}