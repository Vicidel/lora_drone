#include "server.h"

static mav mav;

int main (int argc, char *argv[]) {

	int    len, rc, on = 1;
	int    listen_sd = -1, new_sd = -1;
	int    end_server = false, compress_array = false;
	mav.close_conn = true;
	mav.cloud_ctrl = -1;
	unsigned char   buffer[BUFFERSIZE];
	struct sockaddr_in6   addr;
	int    timeout = -69; // negatif -> infinite loop
	struct pollfd fds[200];
	int    nfds = 1, current_size = 0, i, j;

  	// init pthread
    pthread_t nav_thread;
    pthread_create (&nav_thread, NULL, nav, (void*)&mav);

	listen_sd = socket(AF_INET6, SOCK_STREAM, 0);
	if (listen_sd < 0) {
		perror("socket() failed");
		exit(-1);
	}


	// Allow socket descriptor to be reuseable
	rc = setsockopt(listen_sd, SOL_SOCKET,  SO_REUSEADDR,
	              (char *)&on, sizeof(on));

	if (rc < 0) {
	    perror("setsockopt() failed");
	    close(listen_sd);
	    exit(-1);
	}

	//set nonblocking socket
	rc = ioctl(listen_sd, FIONBIO, (char *)&on);
	if (rc < 0) {
	    perror("ioctl() failed");
	    close(listen_sd);
	    exit(-1);
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin6_family      = AF_INET6;
	memcpy(&addr.sin6_addr, &in6addr_any, sizeof(in6addr_any));
	addr.sin6_port        = htons(SERVER_PORT);
	rc = bind(listen_sd,(struct sockaddr *)&addr, sizeof(addr));

	if (rc < 0) {
	    perror("bind() failed");
	    close(listen_sd);
	    exit(-1);
	}

	rc = listen(listen_sd, 32);
	if (rc < 0) {
	    perror("listen() failed");
	    close(listen_sd);
	    exit(-1);
	}

  	memset(fds, 0 , sizeof(fds));

  	// set poll struct settings
	fds[0].fd = listen_sd;
	fds[0].events = POLLIN;

	do {
	    //printf("Waiting on poll()...\n");
	    rc = poll(fds, nfds, timeout);

	    if (rc < 0) {
			perror("  poll() failed");
			break;
	    } else if (rc == 0){
			printf("  poll() timed out.  End program.\n");
			break;
	    }

	    current_size = nfds;
	    for (i = 0; i < current_size; i++) {
			if(fds[i].revents == 0)
				continue;

			if(fds[i].revents != POLLIN) {
				printf("  Error! revents = %d\n", fds[i].revents);
				end_server = true;
				break;
			}

			if (fds[i].fd == listen_sd) {
				do {
					new_sd = accept(listen_sd, NULL, NULL);
					if (new_sd < 0) {
						if (errno != EWOULDBLOCK) {
							perror("  accept() failed");
							end_server = true;
						}
						break;
					}
					fds[nfds].fd = new_sd;
					fds[nfds].events = POLLIN;
					nfds++;
				} while (new_sd != -1);
			}
			else {
				mav.close_conn = false;

				while(1) {
					
					rc = recv(fds[i].fd, buffer, sizeof(buffer), 0);
					if (rc < 0) {
						if (errno != EWOULDBLOCK) {
							perror("  recv() failed");
							mav.close_conn = true;
						}
						break;
					}

					if (rc == 0) {
						//printf("  Connection closed\n");
						mav.close_conn = true;
						break;
					}

					extract_data(buffer, rc);

					
					if(mav.cloud_ctrl == -1){
						do {
							sleep(1);
						} while(mav.cloud_ctrl == -1);
					}

					if(mav.cloud_ctrl){
						compute_control();
						len = mixer_encode(buffer, BUFFERSIZE);
						rc = send(fds[i].fd, buffer, len, 0);
						//printf("rc %d\n", rc);
					}
					else { // ie rpi ctrl
						len = encode_waypoint(buffer, BUFFERSIZE);
						rc = send(fds[i].fd, buffer, len, 0);
					}
                 
					if (rc < 0) {
						perror("  send() failed");
						mav.close_conn = true;
						break;
					}
				}

				// close file descriptor
				if (mav.close_conn) {
					close(fds[i].fd);
					fds[i].fd = -1;
					compress_array = true;
				}
			}
	    }

	    if (compress_array) {

			compress_array = false;
			for (i = 0; i < nfds; i++) {
				if (fds[i].fd == -1) {
					for(j = i; j < nfds; j++) {
						fds[j].fd = fds[j+1].fd;
					}

					i--;
					nfds--;
				}
			}
	    }
	} while (end_server == false);

	// clean up all of the sockets that are open
	for (i = 0; i < nfds; i++) {
		if(fds[i].fd >= 0)
			close(fds[i].fd);
		}

	return 0;
}

// extract the current msg received into 2 msg buffers
void extract_data(unsigned char* buffer, int len){

	unsigned char attitude_hex[MESSAGE_LEN];
	unsigned char position_hex[MESSAGE_LEN];
	
	if(len != (2*MESSAGE_LEN+1)){
		printf("Transmission error, received %d bytes\n", len);
	} else {
		// extract message id
		// unsigned char id = buffer[2*MESSAGE_LEN];
		// printf("%s\n",id);
		// extract attitude & local position
		for(int i=0; i<MESSAGE_LEN; i++){
			attitude_hex[i] = buffer[i];
			position_hex[i] = buffer[i + MESSAGE_LEN];
		}
		check_msg(attitude_hex, MESSAGE_LEN);
		check_msg(position_hex, MESSAGE_LEN);
	}
}

// put the mavlink msg into its structure
void check_msg(unsigned char* msg, int len) {

	int success = checksum(msg, len);
	if(!success){
		printf("Checksum returned error\n");
	}

	switch(msg[5]){

		case ATTITUDE:
			hex_to_attitude(msg,mav.attitude);
			//printf("ATTITUDE\n" );
			//printf("roll: %f\n",mav.attitude.roll);
			//printf("pitch: %f\n",mav.attitude.pitch);
			//printf("yaw: %f\n",mav.attitude.yaw);
			break;

		case LOCAL_POSITION_NED:
			hex_to_local_position(msg,mav.local_position_ned);
			//printf("LOCAL_POSITION_NED\n" );
			//printf("x: %f\n",mav.local_position_ned.x);
			//printf("y: %f\n",mav.local_position_ned.y);
			//printf("z: %f\n",mav.local_position_ned.z);
			break;

		default:
			//printf("MESSAGE NOT RECOGN\n" );
			break;
	}
}

// Call PID computation if we got the GPS lock, else fill mixer with 0
void compute_control(){

	static int init = 0;
	if(!init && (fabs(mav.local_position_ned.x) > GPS_CHECK)){
		init = 1;
		mav.initial_pos = mav.local_position_ned;
	}

	if(init){
		pid(mav);
		//printf("roll: %f pitch: %f yaw: %f throttle: %f\n", mav.mixer.controls[0], mav.mixer.controls[1], mav.mixer.controls[2], mav.mixer.controls[3]);
	}
	else{
		for(int i=0;i<4;i++){
			mav.mixer.controls[i]=0;
		}
	}
}

// encode mixer values in buffer that will be send over TCP to the rpi
int mixer_encode(unsigned char* buffer, int buffersize){

	conv temp[4];
	int len = 0;
	memset(buffer, 0 , buffersize);

	for(int i=0;i<4;i++){
		temp[i].f = mav.mixer.controls[i];
		my_sprintf(temp[i].s, sizeof(float), &buffer[4*i]);
		len = len+4;
	}

	if(len > buffersize){
		printf("The buffer is too small to encode mixer commands.\n");
	}

	//for (int i=0;i<len;i++){
	//	printf("%02x",buffer[i]);
	//}
	return len;
}

int encode_waypoint(unsigned char* buffer, int buffersize){
	conv temp[3];
	int len = 0;
	memset(buffer, 0 , buffersize);
	for(int i=0;i<3;i++){
		temp[i].f = mav.setpoint[i];
		my_sprintf(temp[i].s, sizeof(float), &buffer[4*i]);
		len = len+4;
	}

	if(len > buffersize){
		printf("The buffer is too small to encode mixer commands.\n");
	}

	//for (int i=0;i<3;i++){
	//	printf("%f",mav.setpoint[i]);
	//} printf("\n");

	//for (int i=0;i<len;i++){
	//	printf("%02x",buffer[i]);
	//}
	return len;
}