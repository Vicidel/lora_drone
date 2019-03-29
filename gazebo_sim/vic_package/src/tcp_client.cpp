#include "tcp_client.h"

// for now a dummy version
void send_tcp(){
    printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    int sock;
    struct sockaddr_in server;
    unsigned char message[64] = "GET /json_print HTTP/1.1";    
    unsigned char server_reply[1024];
    bool connected = false;
    int answer;

    while(1){
    	sleep(1);

    	sock = socket(AF_INET, SOCK_STREAM, 0);
    	if(sock == -1)
			printf("ERROR: could not open socket\n");

        server.sin_addr.s_addr = inet_addr( SERVER_IP );
        server.sin_family = AF_INET;
        server.sin_port = htons( SERVER_SOCKET );

        // connect to remote server
        if(connect(sock, (struct sockaddr *)&server , sizeof(server)) < 0) {
            printf("ERROR: connection to remote server failed\n");
            sleep(5);
            close(sock);
        }
        else{
            connected = true;
            printf("Connected!\n");
            break;
        }
    }

    // send message to server
    if(send(sock, message, sizeof(message), 0) < 0){
    	printf("ERROR: send failed\n");
    }

    // receive a reply from server
    if((answer = recv(sock , server_reply , 1024 , 0)) < 0){
        printf("ERROR: receive failed (length %d)\n", answer);
    }
}