/*
File: server_app.h
Author: Victor Delafontaine
Date: May 2019

Define the URL on which to post different messages as well as the different functions.
*/

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


// URL for POST on server app
#define DRONE_SEND_CURRENT_STATE_URL "http://victor.scapp.io/drone/receive_state"		// ask for instrcutions from server app
#define DRONE_SERVER_URL "http://victor.scapp.io/drone/receive_state_latlng"		// ask for instrcutions from server app
#define DRONE_STATUS_URL "http://victor.scapp.io/param/check_offboard"					// check state from Firebase GUI through server app
#define DRONE_KILL_URL "http://victor.scapp.io/param/check_kill"						// check kill switch from Firebase GUI through server app
#define FIREBASE_EMPTY_URL "http://victor.scapp.io/firebase/empty" 						// empties the Firebase through server app
#define FIREBASE_STORE_HOME_URL "http://victor.scapp.io/firebase/store_home"			// store home in Firebase through server app
#define FIREBASE_STORE_GPS_URL "http://victor.scapp.io/firebase/store_GPS"				// store drone state in Firebase through server app

// store on Firebase
void empty_firebase(int drone_id);
void send_GPS_firebase(double latitude, double longitude, double altitude, double time, 
	int drone_id, std::string state);
void send_home_firebase(double latitude, double longitude, double altitude, double delta_x, 
	double delta_y, double delta_z, double time, int drone_id);

// check for start or kill
int check_offboard_server(int drone_id);
int check_kill_server(void);

// get instructions from server app
std::string send_drone_state(Vector3f position, double time, char* payload, int drone_id, int nb_drone);
std::string send_drone_state_latlng(double lat, double lng, double alt, double time, char* payload, int drone_id, int nb_drone);


#endif