#ifndef TOOLS_H_
#define TOOLS_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>                            
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"

#define BOOT 6
#define ROLL 10
#define X 10
#define PITCH 14
#define Y 14
#define YAW 18
#define Z 18
#define ROLLSPEED 22
#define VX 22
#define PITCHSPEED 26
#define VY 26
#define YAWSPEED 30
#define VZ 30
#define TAU 33000.0f
#define TO_SEC 0.000001

struct Dt{
    uint32_t t1;
    uint32_t t2;
    uint32_t dt;
};

union conv{
	float f;
	unsigned char s[4]; 
};

uint32_t elapsed(const uint32_t t0);
uint32_t get_time_usec();
void dt_(Dt &dt);
int checksum(const unsigned char* msg, int len);
void hex_to_attitude(const unsigned char* msg, mavlink_attitude_t &attitude);
void hex_to_local_position(const unsigned char* msg, mavlink_local_position_ned_t &local_position_ned);
float filter(float current, float last, uint32_t dt);
void my_sprintf(const unsigned char* msg, int len, unsigned char* string);

#endif