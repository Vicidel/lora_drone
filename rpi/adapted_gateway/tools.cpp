#include "tools.h"

uint32_t get_time_usec(){
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

uint32_t elapsed(const uint32_t t0){
	return (get_time_usec() - t0);
}

void dt_(Dt &t){
    t.t2 = get_time_usec();
    t.dt = t.t2 - t.t1;
    t.t1 = t.t2;
}

// TODO
int checksum(const unsigned char* msg, int len){
	int success = 1;
	return success;
}

float filter(float current, float last, uint32_t dt){
	
	float a = (float)dt/TAU;
//printf("a: %f\n", a);
	return (a * current + (1 - a) * last);
}

void my_sprintf(const unsigned char* msg, int len, unsigned char* string){

	for(int i=0; i<len; i++){
		string[i] = msg[i];
	}
} 

void hex_to_attitude(const unsigned char* msg, mavlink_attitude_t &attitude){
	conv temp;
	for(int i=0;i<4;i++){
		temp.s[i] = msg[ BOOT +i];
	}
	attitude.time_boot_ms = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ ROLL +i];
	}
	attitude.roll = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ PITCH +i];
	}
	attitude.pitch = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ YAW +i];
	}
	attitude.yaw = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ ROLLSPEED +i];
	}
	attitude.rollspeed = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ PITCHSPEED +i];
	}
	attitude.pitchspeed = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ YAWSPEED +i];
	}
	attitude.yawspeed = temp.f;
}

void hex_to_local_position(const unsigned char* msg, mavlink_local_position_ned_t &local_position_ned){
	conv temp;
	for(int i=0;i<4;i++){
		temp.s[i] = msg[ BOOT +i];
	}
	local_position_ned.time_boot_ms = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ X +i];
	}
	local_position_ned.x = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ Y +i];
	}
	local_position_ned.y = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ Z +i];
	}
	local_position_ned.z = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ VX +i];
	}
	local_position_ned.vx = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ VY +i];
	}
	local_position_ned.vy = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ VZ +i];
	}
	local_position_ned.vz = temp.f;
}

void hex_to_mixer(const unsigned char* msg, mavlink_set_actuator_control_target_t &mixer) {
	conv temp;
	int offset = sizeof(float);

	for(int i=0;i<4;i++){
		temp.s[i] = msg[i];
	}
	mixer.controls[0] = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ offset +i];
	}
	mixer.controls[1] = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ 2*offset +i];
	}
	mixer.controls[2] = temp.f;

	for(int i=0;i<4;i++){
		temp.s[i] = msg[ 3*offset +i];
	}
	mixer.controls[3] = temp.f;
}

void hex_to_waypoint(const unsigned char* msg, float waypoint[3]){
	conv temp;
	for(int j=0;j<3;j++){
		for(int i=0;i<4;i++){
			temp.s[i] = msg[4*j+i];
		}
		waypoint[j] = temp.f;
	}
}
