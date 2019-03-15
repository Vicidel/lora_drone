#include "tcp_client.h"

// thread to exchange with the cloud server
void* send_tcp(void* arg)
{
    mav &data = *(static_cast<mav *>(arg));
    int sock, len;
    struct sockaddr_in server;
    unsigned char mavlink_messages[2*MESSAGE_LEN+1];
    unsigned char server_reply[1024];
    bool connected = false;
    static unsigned char msg_count[1];

    while(1){

        Dt t;
         
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1) {
            printf("Could not create socket");
        }
        
        server.sin_addr.s_addr = inet_addr( SERVER_IP );
        server.sin_family = AF_INET;
        server.sin_port = htons( SERVER_SOCKET );

        //Connect to remote server
        if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0) {
            perror("Connection to remote server failed");
            sleep(1);
            printf("\nReconnecting in 5s.\n");
            sleep(5);
            close(sock);
        }
        else{
            sleep(2);
            connected = true;
            printf("Connected.\nReady for cloud control.\n\n");
        }
        while(connected){

            // init chrono
            dt_(t);
            my_sprintf(data.hex_attitude, MESSAGE_LEN, &mavlink_messages[0]);
            my_sprintf(data.hex_local_position_ned, MESSAGE_LEN, &mavlink_messages[MESSAGE_LEN]);
            my_sprintf(msg_count, 1, &mavlink_messages[2*MESSAGE_LEN]);

            //Send some data
            if( send(sock , mavlink_messages, sizeof(mavlink_messages) , 0) < 0) {
                puts("Send failed");
                return 0;
            }
             
            //Receive a reply from the server
            if( (len = recv(sock , server_reply , 1024 , 0)) < 0) {
                puts("recv failed");
                return 0;
            }



            if (len == 0) {
                printf("Remote connexion closed.\nFailsafe enabled.");
                connected = false;
                close(sock);
                break;
            }

            // file mixer struct and prepare for sending
            hex_to_mixer(server_reply, data.mixer);

            // write data to UART
            write_mixer(data.fd);

            // for the server to keep track of dataloss
            msg_count[0]++;
        }
    }
}