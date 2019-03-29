#include "server_app.h"



int send_GPS(Vector3f position, float time){
    printf("In %s file, %s function\n", __FILE__, __FUNCTION__);

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_pos_x[10]; sprintf(str_pos_x, "%f", position(0));
    char str_pos_y[10]; sprintf(str_pos_y, "%f", position(1));
    char str_pos_z[10]; sprintf(str_pos_z, "%f", position(2));
    char str_time[10]; sprintf(str_time, "%f", time);
    cJSON_AddStringToObject(root, "pos_x", str_pos_x);
    cJSON_AddStringToObject(root, "pos_y", str_pos_y);
    cJSON_AddStringToObject(root, "pos_z", str_pos_z);
    cJSON_AddStringToObject(root, "time", str_time);
	json = cJSON_PrintUnformatted(root);
	printf("String to print: %s\n", json);

    // create curl object
	CURL *curl = NULL;
	CURLcode res = CURLE_FAILED_INIT; 
	struct curl_slist *headers = NULL;

	// init
	curl_global_init(CURL_GLOBAL_ALL);

	// create the curl message
	curl = curl_easy_init();
	if(curl) {
		// appends header
		headers = curl_slist_append(headers, "Expect:");
		headers = curl_slist_append(headers, "Content-Type: application/json");
		curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

		// URL of server app
		curl_easy_setopt(curl, CURLOPT_URL, "http://victor.scapp.io/gps_coordinates");
		curl_easy_setopt(curl, CURLOPT_POST, 1L);

		// create POST message
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);

		// resquest for return code
		res = curl_easy_perform(curl);

		// check received string for error
		if(res != CURLE_OK)
			fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

		// cleanup
		curl_easy_cleanup(curl);
	}
	curl_global_cleanup();
	return 0;
}


/*// for now a dummy version
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
}*/