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
#define DRONE3_POST_GPS_URL "http://victor.scapp.io/drone3/receive_message"
#define DRONE_POST_HOME_FIREBASE_URL "http://victor.scapp.io/drone/store_home_firebase"
#define DRONE_POST_GPS_FIREBASE_URL "http://victor.scapp.io/drone/store_GPS_firebase"
#define DRONE_STATUS_URL "http://victor.scapp.io/param/drone_ready_to_takeoff"

std::string send_GPS(Vector3f position, double time, char* payload);
std::string send_GPS_drone3(Vector3f position, double time, char* payload, int no_drone);
void send_GPS_firebase(Vector3f position, double time);
void send_home_firebase(double latitude, double longitude, double altitude, double delta_x, double delta_y, double delta_z, double time);
int check_server(int no_drone);

#endif