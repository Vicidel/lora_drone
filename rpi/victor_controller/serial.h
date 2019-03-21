#ifndef SERIAL_DECODE_H_
#define SERIAL_DECODE_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>         
#include <fcntl.h>                 
#include <termios.h>       
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <math.h>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"
#include "tools.h"
#include "pid.h"
#include "tcp_client.h"

#define WRITE_FREQ 5000 // in micro seconds
#define BUFFER_SIZE 5000
#define MESSAGE_LEN 36
#define ATTITUDE 30
#define LOCAL_POSITION_NED 32
#define SUCCESS 1
#define WRITE_FRAME_SIZE 51
#define INIT 1000000
#define GPS_CHECK 0.00001f

// struct contains the messages we need
struct mav{
	mavlink_local_position_ned_t local_position_ned;
	mavlink_local_position_ned_t initial_pos;
	float way_point[3];
	mavlink_attitude_t attitude;
	mavlink_set_actuator_control_target_t mixer;
	unsigned char hex_local_position_ned[MESSAGE_LEN];
	unsigned char hex_attitude[MESSAGE_LEN];
	int sysid;
	int autopilot_id;
	int compid;
	int offboard;
	int gps_lock;
};

void enable_offboard_control(const int fd );
void arm(const int fd);
void write_mixer(const int fd);
void run(const int fd);
void setup_usart(int &fd);
void check_msg(unsigned char* msg, int len);
int decode(unsigned char* check_buffer, int len);

#endif
