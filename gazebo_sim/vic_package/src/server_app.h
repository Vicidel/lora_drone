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


#define DRONE_STATUS_URL "http://victor.scapp.io/param/drone_ready_to_takeoff"
#define FIREBASE_EMPTY_URL "http://victor.scapp.io/firebase/empty"
#define FIREBASE_STORE_HOME_URL "http://victor.scapp.io/firebase/store_home"
#define FIREBASE_STORE_GPS_URL "http://victor.scapp.io/firebase/store_GPS"

void send_GPS_firebase(double latitude, double longitude, double altitude, double time, int drone_id);
void send_home_firebase(double latitude, double longitude, double altitude, double delta_x, double delta_y, double delta_z, double time);
int check_server(int drone_id);
void empty_firebase(void);

// for mission v2
#define DRONE_SEND_CURRENT_STATE_URL "http://victor.scapp.io/drone/receive_state"
std::string send_drone_state(Vector3f position, double time, char* payload, int drone_id, int nb_drone);

#endif