#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

// standard libraries
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

// for server communication
#include <curl/curl.h>
#include "cJSON.h"

// libraries for Vector3f
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation


#define DRONE_POST_GPS_URL "http://victor.scapp.io/drone/receive_message"
#define DRONE_GET_WAYPOINT_URL "http://victor.scapp.io/drone/send_waypoint"

std::string send_GPS(Vector3f position, double time, char* payload);
std::string send_GPS_drone3(Vector3f position, double time, char* payload, int no_drone);


#endif