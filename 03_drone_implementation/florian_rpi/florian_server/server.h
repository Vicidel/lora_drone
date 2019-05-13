#ifndef SERVER_H_
#define SERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"
#include "navigator.h"
#include "tools.h"
#include "pid.h"

#define SERVER_PORT  10005

#define ATTITUDE 30
#define LOCAL_POSITION_NED 32
#define MESSAGE_LEN 36
#define BUFFERSIZE 256

#define GPS_CHECK 0.00001f

struct mav{
	mavlink_local_position_ned_t local_position_ned;
	mavlink_local_position_ned_t initial_pos;
	mavlink_attitude_t attitude;
	mavlink_set_actuator_control_target_t mixer;
	int cloud_ctrl;
	float setpoint[3];
	int close_conn;
};

void extract_data(unsigned char* buffer, int len);
void check_msg(unsigned char* msg, int len);
void compute_control();
int mixer_encode(unsigned char* buffer, int buffersize);
int encode_waypoint(unsigned char* buffer, int buffersize);

#endif