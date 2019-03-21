#ifndef PID_H_
#define PID_H_

#include "tools.h"

#define VEL_MAX 2
#define VEL_MIN -2
#define MAX_ANGLE 1.3
#define STICK_OFFSET 0.5
#define PI 3.14159265
#define TAKEOFF 3000000

// PID gains
#define MC_X_P 1.0
#define MC_Y_P 1.0
#define MC_Z_P 1.0
#define MC_VX_P 0.1
#define MC_VX_I 0.01
#define MC_VX_D 0.001
#define MC_VY_P 0.1
#define MC_VY_I 0.01
#define MC_VY_D 0.001
#define MC_VZ_P 0.5
#define MC_VZ_I 0.01
#define MC_VZ_D 0.001

// PID gains from px4
#define MC_ROLL_P 2.0
#define MC_ROLLRATE_P 0.15
#define MC_ROLLRATE_I 0.05
#define MC_ROLLRATE_D  0.000001
#define MC_PITCH_P 2.0
#define MC_PITCHRATE_P 0.15
#define MC_PITCHRATE_I 0.05
#define MC_PITCHRATE_D 0.000001
#define MC_YAW_P 0.5
#define MC_YAWRATE_P 0.1
#define MC_YAWRATE_I 0.0
#define MC_YAWRATE_D 0.0

struct Pos_stamp
{
	uint32_t last_t;
};

struct Error_vel
{
	float last_vx;
	float int_vx;
	float der_vx;

	float last_vy;
	float int_vy;
	float der_vy;

	float last_vz;
	float int_vz;
	float der_vz;

	uint32_t last_t;
	uint32_t dt;
};

struct Error_rate
{
    float last_pitch;
    float int_pitch;
    float der_pitch;

    float last_roll;
    float int_roll;
    float der_roll;

    float last_yaw;
    float int_yaw;
    float der_yaw;
};

struct Torque
{
	float roll;
	float pitch;
	float yaw;
	float throttle;
};

void pid(mavlink_local_position_ned_t initial_pos, mavlink_attitude_t attitude, mavlink_local_position_ned_t drone, mavlink_set_actuator_control_target_t &mixer, const float way_point[3]);
void pid_pos_ctrl(float p_ref[4], mavlink_attitude_t attitude, mavlink_local_position_ned_t drone, uint32_t stamp, mavlink_set_actuator_control_target_t &mixer);
void pid_vel_ctrl(float v_ref[4], mavlink_attitude_t attitude, mavlink_local_position_ned_t drone, uint32_t stamp, mavlink_set_actuator_control_target_t &mixer);
void pid_angle_ctrl(float a_ref[3], mavlink_attitude_t attitude, uint32_t dt, mavlink_set_actuator_control_target_t &mixer);
void pid_rate_ctrl(float ref[3], mavlink_attitude_t attitude, uint32_t error_dt, mavlink_set_actuator_control_target_t &mixer);

#endif
