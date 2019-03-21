#include "serial.h"

static mav mav;


// create the serial link between the RPi and PX4
void setup_usart(int &fd){

	// open in non blocking read/write mode
    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1){
        printf("ERROR: unable to open serial port.\n");
    }
    else{
    	printf("Serial port opened.\n");
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
}

// arm the drone
void arm(const int fd)
{
	// prepare command for arming
	mavlink_command_long_t com = { 0 };
	com.target_system    = 1;
	com.target_component = mav.autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = 1; // 1 to arm, 0 to disarm

	// encode message
	mavlink_message_t message;
	char buff[200];
	mavlink_msg_command_long_encode(com.target_system, com.target_component, &message, &com);
	
	// translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);
	
	// write 
	int bytes_written = write(fd, (void*)buff, len);

	// check the write
	if(bytes_written <= 0 )
		fprintf(stderr,"WARNING: could not send arm command \n");
}

// decode a buffer and analyse if messages begin by 0xFEC1
int decode(unsigned char* check_buffer, int len){
	
	int success = 0;
	int msg_len = 0;
	int j		= 0;
	
	// message char vector
	unsigned char message[MESSAGE_LEN];
	
	printf("Received buffer of length %d to decode\nBuffer:", len);
	
	// looking for messages starting by 0xFEC1
	for (int i=0; i<len-1; i++){
		
		printf("%02X", check_buffer[i]);
		
		// found 0xFEC1
		if((check_buffer[i] == 0xFE) && (check_buffer[i+1] == 0x1C)){
			msg_len = len - i;

			if(msg_len >= MESSAGE_LEN){
				printf("Decoding:\n");
				
				// full message available, file it
				for(j=0; j<MESSAGE_LEN; j++){
					message[j] = check_buffer[i+j];
					printf("%02X", message[j]);
				}
				printf("\n\n");
				success = len - i - j;
				printf("Success: %d\n", success);
				
				// send message for checking
				check_msg(message, MESSAGE_LEN);
				return success;
			}
		}
	}
	printf("\n");
	return success;
}

void check_msg(unsigned char* msg, int len){
/*	// this function could eventally detect different message sizes
	// right now it only looks for Attitude and Local_Position_Ned msgs
	static uint32_t count;
	count++;
	printf("\n count: %d\n", count);
	
	// currently only 1 len of message, lets check it is right

	static int init_pos;

	int success = checksum(msg, len);
	if(!success){
		printf("Checksum returned error\n");
	}

	//static Dt t;

	printf("\n");
	switch(msg[5]){

		case ATTITUDE:
			my_sprintf(msg, len, mav.hex_attitude);

			hex_to_attitude(msg,mav.attitude);
			printf("%f %f %f\n", mav.attitude.rollspeed,mav.attitude.pitchspeed,mav.attitude.yawspeed);
			printf("packet: %02X\n",msg[2]);
			printf("ATTITUDE\n" );
			printf("roll: %f\n",attitude.roll);
			printf("pitch: %f\n",attitude.pitch);
			printf("yaw: %f\n",attitude.yaw);
			break;

		case LOCAL_POSITION_NED:
			my_sprintf(msg, len, mav.hex_local_position_ned);
			for(int i=0;i<len;i++){
            	printf("%02X", mav.hex_local_position_ned[i]);
        	}
        	printf("\n");
        	for(int i=0;i<len;i++){
            	printf("%02X", msg[i]);
        	}
        	printf("\n");
			hex_to_local_position(msg,mav.local_position_ned);
			printf("%f %f %f\n", mav.local_position_ned.x,mav.local_position_ned.y,mav.local_position_ned.z);
			printf("packet: %02X\n",msg[2]);
			dt_(t);
			printf("time read att: %d\n", t.dt);
			printf("LOCAL_POSITION_NED\n" );
			printf("x: %f\n",local_position_ned.x);
			printf("y: %f\n",local_position_ned.y);
			printf("z: %f\n",local_position_ned.z);
			break;

		default:
			printf("MESSAGE NOT RECOGN\n" );
			break;
	}

	if((fabs(mav.local_position_ned.x) > GPS_CHECK) && !init_pos){
		mav.offboard = 1;
		init_pos = 1;
		mav.initial_pos = mav.local_position_ned;
		printf("init x:%f y:%f z:%f\n", mav.local_position_ned.x ,mav.local_position_ned.y ,mav.local_position_ned.z);
	}*/
}


// main code
void run(const int fd){

	// init pthread
    pthread_t tpc_thread;
    pthread_create(&tpc_thread, NULL, send_tcp, (void*)&mav);

	while(1){
		
		if (fd != -1){
			// buffer that store input read
			unsigned char rx_buffer[256];
			int buff_len;
			int new_ptr;

			// extended buffer that store everything
			static unsigned char check_buffer[BUFFER_SIZE];
			static int indice;

			// store input read on serial
			buff_len = read(fd, (void*)rx_buffer, 255);
			if (buff_len < 0) {
				// an error occured (will occur if there are no bytes)
				printf("ERROR: buffer length is <1, no byte received \n");
			}
			else if (buff_len == 0){
				// no data waiting
			}
			else{
				printf("Length of received buffer = %d\n\n\n", buff_len);
				
				// add input in second buffer
				for(int i=0; i<buff_len; i++){
					check_buffer[indice + i] = rx_buffer[i];
				}
				
				// update check_buffer length
				if((indice + buff_len) < BUFFER_SIZE)
					indice = indice + buff_len;
				else
					indice = 0;
				
				for(int k=0; k<indice; k++){
					printf("%02X",check_buffer[k]);
				}
				printf("\n");

				if((new_ptr = decode(check_buffer, indice))){
					// msg found, we can reset the buffer at correct place
					for(int u=0; u < new_ptr; u++){
						check_buffer[u] = check_buffer[indice - new_ptr + u];
						printf("%02X",check_buffer[u]);
					}
					indice = new_ptr;
					printf("\nPointer new_ptr: %d\n", new_ptr);
				}
			}
		}
	}
}
