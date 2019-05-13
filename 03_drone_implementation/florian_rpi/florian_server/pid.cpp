#include "pid.h"

static float antiwindup;

void pid(mav &mavlink){
	
	uint32_t stamp = get_time_usec();
	// position reference {x,y,z,yaw} in local NED referentiel
	float p_ref[4];

	// still landed, avoid overflow of integrator
	if(!(mavlink.setpoint[0] && mavlink.setpoint[1] && mavlink.setpoint[2])){
		antiwindup = 0.1;
	}
	else{
		antiwindup = 1;
	}

	p_ref[0] = mavlink.setpoint[0];
	p_ref[1] = mavlink.setpoint[1];
	p_ref[2] = mavlink.setpoint[2];
	p_ref[3] = 0;

	pid_pos_ctrl(p_ref, mavlink.attitude, mavlink.local_position_ned, stamp, mavlink.mixer);
	//printf("x: %f\ty: %f\tz: %f\tvx: %f\tvy: %f\tvz: %f\n", mav.local_position_ned.x, mav.local_position_ned.y, mav.local_position_ned.z, mav.local_position_ned.vx, mav.local_position_ned.vy, mav.local_position_ned.vz);
	//printf("x: %f\t y: %f\t z:%f\n", mav.attitude.roll, mav.attitude.pitch, mav.attitude.yaw);
}

void pid_pos_ctrl(float p_ref[4], mavlink_attitude_t attitude, mavlink_local_position_ned_t drone, uint32_t stamp, mavlink_set_actuator_control_target_t &mixer) {

	static Pos_stamp controller_stamp;

	if(stamp != controller_stamp.last_t) {

		float v_ref[4];

		// update stamp
		controller_stamp.last_t = stamp;

		// define a simple proportional velocity converter
		v_ref[0] = MC_X_P * (p_ref[0] - drone.x);
		v_ref[1] = MC_Y_P * (p_ref[1] - drone.y);
		v_ref[2] = MC_Z_P * (p_ref[2] + drone.z); // reverse axis in NED

		v_ref[3] = p_ref[3]; // yaw not controlled here
		
		// go to cascaded velocity controller
		pid_vel_ctrl(v_ref, attitude, drone, stamp, mixer);
	}
}

void pid_vel_ctrl(float v_ref[4], mavlink_attitude_t attitude, mavlink_local_position_ned_t drone, uint32_t stamp, mavlink_set_actuator_control_target_t &mixer) {
	
	static Error_vel error;
	static Dt t;
	Torque torque;
	// angle reference
	float pitch;
	float roll;
	float a_ref[3];
	float angle = attitude.yaw;

	// check for init to avoid overflow at loading
	if(error.last_t == 0){
		error.last_t = stamp;
		dt_(t);
		// set to hovering mixer
		mixer.controls[3] = STICK_OFFSET;
		pid_angle_ctrl(a_ref, attitude, error.dt, mixer);
	}
	else {

		// update time in micro second
		dt_(t);
		error.dt = t.dt;
		error.last_t = stamp;

		// compute velocity error
		float error_vx = v_ref[0] - drone.vx;
		float error_vy = v_ref[1] - drone.vy;
		float error_vz = v_ref[2] + drone.vz; // reverse axis in NED

		// update ingral errors
		error.int_vx = error.int_vx + error_vx;
		error.int_vy = error.int_vy + error_vy;
		error.int_vz = error.int_vz + error_vz;

		// bound the integrator
		if(fabs(error.int_vx) > antiwindup){
			error.int_vx = 0;
		}
		if(fabs(error.int_vy) > antiwindup){
			error.int_vy = 0;
		}
		if(fabs(error.int_vz) > antiwindup){
			error.int_vz = 0;
		}

		//printf("vx:%f,vy:%f,vz:%f\n",error.int_vx,error.int_vy,error.int_vz);

		// compute exponential smoothing filter for derivative part
		error.der_vx = filter(error_vx, error.last_vx, error.dt);
		error.der_vy = filter(error_vy, error.last_vy, error.dt);
		error.der_vz = filter(error_vz, error.last_vz, error.dt);

		// convert us to second
		float dt = (float)error.dt * TO_SEC;

		// set velocity projection to angle PID 
		float vx_to_angle = MC_VX_P * error_vx + MC_VX_I * error.int_vx * dt + MC_VX_D * error.der_vx / dt;
		float vy_to_angle = MC_VY_P * error_vy + MC_VY_I * error.int_vy * dt + MC_VY_D * error.der_vy / dt;
		float vz_to_angle = MC_VZ_P * error_vz + MC_VZ_I * error.int_vz * dt + MC_VZ_D * error.der_vz / dt;

		// TODO add projection on Z ?
		torque.throttle = vz_to_angle + STICK_OFFSET;

		if (torque.throttle >= 1) {
			torque.throttle = 1;
		}
		else if (torque.throttle <= 0.2) {
			torque.throttle = 0.2;
		}

		// update D last_v terms
		error.last_vx = error_vx;
		error.last_vy = error_vy;
		error.last_vz = error_vz;


		// from global NED to body frame axis transformation
		roll =  (vy_to_angle - tan(angle)*vx_to_angle)/(sin(angle)*tan(angle)+cos(angle));
		pitch =  (- vx_to_angle - roll*sin(angle))/(cos(angle));

		//TODO add vector Accel representation
		// -> transform max_angle into 1 projection onto XY plane
		// -> add correction of the Z axis due to the drone's tilt angle

		// set roll and pitch bondary reference to avoid fliping
		if(roll > MAX_ANGLE){
			roll = MAX_ANGLE;
		} else if(roll < -MAX_ANGLE){
			roll = -MAX_ANGLE;
		}

		if(pitch > MAX_ANGLE){
			pitch = MAX_ANGLE;
		} else if(pitch < -MAX_ANGLE){
			pitch = -MAX_ANGLE;
		}

		// set angle table reference for cascaded attitude controller
		a_ref[0] = roll;
		a_ref[1] = pitch;
		a_ref[2] = v_ref[3]; // yaw not controlled here

		// set throttle
		mixer.controls[3] = torque.throttle;

		// go to euler angles controller
		pid_angle_ctrl(a_ref, attitude, error.dt, mixer);
	}
}


void pid_angle_ctrl(float a_ref[3], mavlink_attitude_t attitude, uint32_t dt, mavlink_set_actuator_control_target_t &mixer){

	float ref[3];

//printf("r:%f p:%f\n", a_ref[0],a_ref[1]);

	ref[0] = MC_ROLL_P * (a_ref[0] - attitude.roll);
	ref[1] = MC_PITCH_P * (a_ref[1] - attitude.pitch);
	ref[2] = MC_YAW_P * (a_ref[2] - attitude.yaw);

	// go to rate controller
	pid_rate_ctrl(ref, attitude, dt, mixer);
}

void pid_rate_ctrl(float ref[3], mavlink_attitude_t attitude, uint32_t error_dt, mavlink_set_actuator_control_target_t &mixer){

	static Error_rate error;
	Torque torque;

	// init case
	if(error_dt == 0){
		mixer.controls[0] = 0;
		mixer.controls[1] = 0;
		mixer.controls[2] = 0;
	}
	else {
		// compute rate error
		float error_roll = ref[0] - attitude.rollspeed;
		float error_pitch = ref[1] - attitude.pitchspeed;
		float error_yaw = ref[2] - attitude.yawspeed;

		// update ingral errors
		error.int_roll = error.int_roll + error_roll;
		error.int_pitch = error.int_pitch + error_pitch;
		error.int_yaw = error.int_yaw + error_yaw;

		// bound the integrator
		if(fabs(error.int_roll) > antiwindup){
			error.int_roll = 0;
		}
		if(fabs(error.int_pitch) > antiwindup){
			error.int_pitch = 0;
		}
		if(fabs(error.int_yaw) > antiwindup){
			error.int_yaw = 0;
		}

		//printf("roll:%f,pitch:%f,yaw:%f\n",error.int_roll,error.int_pitch,error.int_yaw);

		// compute exponential smoothing filter for derivative part
		error.der_roll = filter(error_roll, error.last_roll, error_dt);
		error.der_pitch = filter(error_pitch, error.last_pitch, error_dt);
		error.der_yaw = filter(error_yaw, error.last_yaw, error_dt);

		// convert us to second
		float dt = (float)error_dt * TO_SEC;

	//printf("err D:%f\tdt:%f \n", error.der_roll, dt);

		// set velocity projection to angle PID 
		torque.roll = MC_ROLLRATE_P * error_roll + MC_ROLLRATE_I * error.int_roll * dt + MC_ROLLRATE_D * error.der_roll / dt;
		torque.pitch = MC_PITCHRATE_P * error_pitch + MC_PITCHRATE_I * error.int_pitch * dt + MC_PITCHRATE_D * error.der_pitch / dt;
		torque.yaw = MC_YAWRATE_P * error_yaw + MC_YAWRATE_I * error.int_yaw * dt + MC_YAWRATE_D * error.der_yaw / dt;

		// update D terms
		error.last_roll = error_roll;
		error.last_pitch = error_pitch;
		error.last_yaw = error_yaw;

	//printf("P:%f\tI:%f\tD:%f\n", MC_ROLLRATE_P * error_roll, MC_ROLLRATE_I * error.int_roll, MC_ROLLRATE_D * error.der_roll / dt);

	//printf("roll = %f, pitch = %f, yaw = %f\n", torque.roll, torque.pitch, torque.yaw);
		
		// limiting mixer input in a range of [-1;1]
		if (torque.roll > 1)
			torque.roll = 1;
		if (torque.roll < -1)
			torque.roll = -1;
		if (torque.pitch > 1)
			torque.pitch = 1;
		if (torque.pitch < -1)
			torque.pitch = -1;
		if (torque.yaw > 1)
			torque.yaw = 1;
		if (torque.yaw < -1)
			torque.yaw = -1;
		
		// update mixer
		mixer.controls[0] = torque.roll;
		mixer.controls[1] = torque.pitch;
		mixer.controls[2] = torque.yaw;
	}
}


