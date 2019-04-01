#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

// standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// for server communication
#include <curl/curl.h>
#include "cJSON.h"

// libraries for Vector3f
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen; // To use matrix and vector representation


int send_GPS(Vector3f position, float time);

#endif