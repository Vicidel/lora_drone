#include "serial.h"

static mav mav;

void setup_usart(int &fd){

	//Open in non blocking read/write mode
    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1){
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.\n");
    }
    else{
    	printf("\nSerial port opened.\n\n");
    }

    // init termios cfg for serial
	struct termios serialSet;
	memset(&serialSet, 0, sizeof(serialSet));
	serialSet.c_iflag = IGNBRK;
	serialSet.c_cflag = CS8 | CREAD | CLOCAL;
	memset(serialSet.c_cc, _POSIX_VDISABLE, NCCS);
	serialSet.c_cc[VMIN] = 0;
	serialSet.c_cc[VTIME] = 0;
	speed_t baudRate = B921600;
	cfsetispeed(&serialSet, baudRate);
	cfsetospeed(&serialSet, baudRate);

	if (tcsetattr(fd, TCSANOW, &serialSet) == -1) 
		printf("Error cfg\n");
	mav.fd = fd;
}


// write mavlink message to uart
void write_mixer(const int fd) {

	mavlink_set_actuator_control_target_t mixer = mav.mixer;

	mixer.target_system = mav.sysid;
	mixer.target_component = mav.autopilot_id;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	char buff[200];
	mavlink_msg_set_actuator_control_target_encode(mav.sysid, mav.compid, &message, &mixer);
	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	if (len != WRITE_FRAME_SIZE){
		printf("writing message length unconsistant\n");
	}

	int bytes_written = write(fd, (void*)buff, WRITE_FRAME_SIZE);

	// check the write
	if ( bytes_written <= 0 )
		fprintf(stderr,"WARNING: could not send SET_ACTUATOR_CONTROL_TARGET \n");
}

// enable offboard mode
void enable_offboard_control(const int fd ){
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = 1;
	com.target_component = mav.autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = 1; // 1 enable offboard (0 dsiable)

	// Encode
	mavlink_message_t message;
	char buff[200];
	mavlink_msg_command_long_encode(com.target_system, com.target_component, &message, &com);
	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	int bytes_written = write(fd, (void*)buff, len);

	// check the write
	if ( bytes_written <= 0 )
		fprintf(stderr,"WARNING: could not send offboard mode command \n");
}


// arm the drone
void arm(const int fd)
{
	// Prepare command for arming
	mavlink_command_long_t com = { 0 };
	com.target_system    = 1;
	com.target_component = mav.autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = 1; // 1 to arm, 0 to disarm

	// Encode
	mavlink_message_t message;
	char buff[200];
	mavlink_msg_command_long_encode(com.target_system, com.target_component, &message, &com);
	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	int bytes_written = write(fd, (void*)buff, len);

	// check the write
	if ( bytes_written <= 0 )
		fprintf(stderr,"WARNING: could not send arm command \n");
}


// decode data from uart and look for the 2 mavlink msg: local_pos_ned and attitude
int decode(unsigned char* check_buffer, int len){

	int success = 0;
	int msg_len = 0;
	int j		= 0;
	unsigned char message[MESSAGE_LEN];

	// look for msg starting by 0x FEC1
	for (int i=0; i<len-1; i++){
		if((check_buffer[i] == 0xFE) && (check_buffer[i+1] == 0x1C)){
			// we found the begin of a message
			// lets check if we got the full message in buffer already
			msg_len = len - i;

			if(msg_len >= MESSAGE_LEN){
				// full msg available, lets file it
				for(j=0; j<MESSAGE_LEN; j++){
					message[j] = check_buffer[i+j];
				}
				success = len - i - j;
				check_msg(message,MESSAGE_LEN);
			}
		}
	}	return success;
}

void check_msg(unsigned char* msg, int len){
	// this function could eventally detect different message sizes
	// right now it only looks for Attitude and Local_Position_Ned msgs
	// currently only 1 len of message


	int success = checksum(msg, len);
	if(!success){
		printf("Checksum returned error\n");
	}

	//static Dt t;

	//printf("\n");
	switch(msg[5]){

		case ATTITUDE:
			my_sprintf(msg, len, mav.hex_attitude);

			hex_to_attitude(msg,mav.attitude);
			//printf("packet: %02X\n",msg[2]);
			//printf("ATTITUDE\n" );
			//printf("roll: %f\n",attitude.roll);
			//printf("pitch: %f\n",attitude.pitch);
			//printf("yaw: %f\n",attitude.yaw);
			break;

		case LOCAL_POSITION_NED:
			my_sprintf(msg, len, mav.hex_local_position_ned);
			//for(int i=0;i<len;i++){
            //	printf("%02X", mav.hex_local_position_ned[i]);
        	//}
        	//printf("\n");
        	//for(int i=0;i<len;i++){
            //	printf("%02X", msg[i]);
        	//}
        	//printf("\n");
			hex_to_local_position(msg,mav.local_position_ned);
			printf("%f %f %f\n", mav.local_position_ned.x,mav.local_position_ned.y,mav.local_position_ned.z);
			//printf("packet: %02X\n",msg[2]);
			//dt_(t);
			//printf("time read att: %d\n", t.dt);
			//printf("LOCAL_POSITION_NED\n" );
			//printf("x: %f\n",local_position_ned.x);
			//printf("y: %f\n",local_position_ned.y);
			//printf("z: %f\n",local_position_ned.z);
			break;

		default:
			//printf("MESSAGE NOT RECOGN\n" );
			break;
	}
}

// main while loop
void run(const int fd){

	// init pthread
    pthread_t tpc_thread;
    pthread_create (&tpc_thread, NULL, send_tcp, (void*)&mav);

	while(1){
		
		if (fd != -1){
			// buffer that store input read
			unsigned char rx_buffer[256];
			int buff_len;
			int new_ptr;

			// the extended buff that store everything
			static unsigned char check_buffer[BUFFER_SIZE];
			static int indice;

			// store input ready on serial
			buff_len = read(fd, (void*)rx_buffer, 255);
			if (buff_len < 0) {
					//An error occured (will occur if there are no bytes)
					printf("ERROR bufferlen <1 \n");
			}
			else if (buff_len == 0){
					//No data waiting
			}
			else{
				// add input in second buffer
				for(int i=0; i<buff_len; i++){
					check_buffer[indice + i] = rx_buffer[i];
				}
				// update check_buffer length
				if((indice + buff_len) < BUFFER_SIZE)
					indice = indice + buff_len;
				else
					indice = 0;

				if((new_ptr = decode(check_buffer,indice))){
					// msg found, we can reset the buffer at correct place
					for(int u=0; u < new_ptr; u++){
						check_buffer[u] = check_buffer[indice - new_ptr + u];
					}
					indice = new_ptr;
				}
			}
		}
	}
}
