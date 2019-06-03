/*
File: server_app.cpp
Author: Victor Delafontaine
Date: May 2019

Creates function for communication with the Swisscom server. 
Sends the current position as JSON file through HTTP POST on the server.
The commands to follow are given in the return string.
*/

#include "server_app.h"



/**************************************************************************
**************************   POST FUNCTIONS   *****************************
***************************************************************************/

// callback for POST
size_t writeFunction(void *ptr, size_t size, size_t nmemb, std::string* data) {
    data->append((char*) ptr, size * nmemb);
    return size * nmemb;
}

// method for cURL POST of JSON file
std::string post_JSON(const char* url, char* json){

    // set response string
    std::string response_string;

    // create curl object
    CURL *curl = NULL;
    CURLcode res = CURLE_FAILED_INIT; 
    struct curl_slist *headers = NULL;

    // init
    curl_global_init(CURL_GLOBAL_ALL);

    // all curl stuff
    curl = curl_easy_init();
    if(curl) {
        // appends header
        headers = curl_slist_append(headers, "Expect:");
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // URL of server app
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFunction);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

        // create POST message
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);

        // make the curl request
        res = curl_easy_perform(curl);

        // check received string for error
        if(res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

        // cleanup
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();

    return response_string;
}

// method for cURL POST of JSON file
void post_JSON_noanswer(const char* url, char* post_array){

    // create a command
    std::string cmd = "curl -X POST -H 'Content-Type: application/json' -d '";
    cmd.append(post_array);
    cmd.append("' '");
    cmd.append(url);
    cmd.append("' > /dev/null 2>&1 &");
    //std::cout << "Produced command: " << cmd << std::endl;

    // execute it shell like
    system(cmd.c_str());
}


/**************************************************************************
***********************   SEND INFO FUNCTIONS   ***************************
***************************************************************************/

// POST the drone xy coordinates and drone number
std::string send_drone_state(Vector3f position, double time, char* payload, 
    int drone_id, int nb_drone, bool bool_no_answer){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_pos_x[10];    sprintf(str_pos_x, "%f", position(0));
    char str_pos_y[10];    sprintf(str_pos_y, "%f", position(1));
    char str_pos_z[10];    sprintf(str_pos_z, "%f", position(2));
    char str_time[30];     sprintf(str_time, "%f", time);
    char str_drone_id[10]; sprintf(str_drone_id, "%d", drone_id);
    char str_nb_drone[10]; sprintf(str_nb_drone, "%d", nb_drone);
    cJSON_AddStringToObject(root, "x", str_pos_x);
    cJSON_AddStringToObject(root, "y", str_pos_y);
    cJSON_AddStringToObject(root, "z", str_pos_z);
    cJSON_AddStringToObject(root, "ts", str_time);
    cJSON_AddStringToObject(root, "str", payload);
    cJSON_AddStringToObject(root, "id", str_drone_id);
    cJSON_AddStringToObject(root, "nb", str_nb_drone);
    json = cJSON_PrintUnformatted(root);
    //std::cout << "String sent:" << json << std::endl;

    if(bool_no_answer==false){
        // POST JSON on URL
        response_string = post_JSON(DRONE_SEND_CURRENT_STATE_URL, json);

        // print for debugging
        std::cout << "POST answer: " << response_string << std::endl;

        return response_string;
    }
    else{
        // POST without answer
        post_JSON_noanswer(DRONE_SEND_CURRENT_STATE_URL, json);
        return "XX no answer required";
    }
}

// POST drone GPS coordinates on Firebase
void send_GPS_firebase(double latitude, double longitude, double altitude, double time, 
    int drone_id, std::string state, double relative_altitude, int fsm_state){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_lat[10]; sprintf(str_lat, "%f", latitude);
    char str_lng[10]; sprintf(str_lng, "%f", longitude);
    char str_alt[10]; sprintf(str_alt, "%f", altitude);
    char str_alt_rel[10]; sprintf(str_alt_rel, "%f", relative_altitude);
    char str_time[30]; sprintf(str_time, "%f", time);
    char str_drone_id[10]; sprintf(str_drone_id, "%d", drone_id);
    char str_fsm_state[10]; sprintf(str_fsm_state, "%d", fsm_state);
    char str_state[20]; strcpy(str_state, state.c_str());
    cJSON_AddStringToObject(root, "lat", str_lat);
    cJSON_AddStringToObject(root, "lon", str_lng);
    cJSON_AddStringToObject(root, "alt", str_alt);
    cJSON_AddStringToObject(root, "rel", str_alt_rel);
    cJSON_AddStringToObject(root, "ts", str_time);
    cJSON_AddStringToObject(root, "id", str_drone_id);
    cJSON_AddStringToObject(root, "fsm", str_fsm_state);
    cJSON_AddStringToObject(root, "state", str_state);
    json = cJSON_PrintUnformatted(root);

    // POST JSON on URL
    post_JSON_noanswer(FIREBASE_STORE_GPS_URL, json);
}

// POST home coordinates 
void send_home_firebase(double latitude, double longitude, double altitude, 
    double delta_x, double delta_y, double delta_z, double time, int drone_id){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_lat[10]; sprintf(str_lat, "%f", latitude);
    char str_lng[10]; sprintf(str_lng, "%f", longitude);
    char str_alt[10]; sprintf(str_alt, "%f", altitude);
    char str_time[30]; sprintf(str_time, "%f", time);
    char str_dx[10]; sprintf(str_dx, "%f", delta_x);
    char str_dy[10]; sprintf(str_dy, "%f", delta_y);
    char str_dz[10]; sprintf(str_dz, "%f", delta_z);
    char str_drone_id[10]; sprintf(str_drone_id, "%d", drone_id);
    cJSON_AddStringToObject(root, "lat", str_lat);
    cJSON_AddStringToObject(root, "lon", str_lng);
    cJSON_AddStringToObject(root, "alt", str_alt);
    cJSON_AddStringToObject(root, "ts", str_time);
    cJSON_AddStringToObject(root, "dx", str_dx);
    cJSON_AddStringToObject(root, "dy", str_dy);
    cJSON_AddStringToObject(root, "dz", str_dz);
    cJSON_AddStringToObject(root, "id", str_drone_id);
    json = cJSON_PrintUnformatted(root);

    // POST JSON on URL
    post_JSON_noanswer(FIREBASE_STORE_HOME_URL, json);
}



/**************************************************************************
***********************   CHECK THINGS FUNCTIONS   ************************
***************************************************************************/

// POST home coordinates 
void empty_firebase(int drone_id){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_drone_id[10]; sprintf(str_drone_id, "%d", drone_id);
    cJSON_AddStringToObject(root, "id", str_drone_id);
    json = cJSON_PrintUnformatted(root);

    // POST JSON on URL
    response_string = post_JSON(FIREBASE_EMPTY_URL, json);
}

// check if start signal sent from server
int check_offboard_server(int drone_id){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_drone_id[10]; sprintf(str_drone_id, "%d", drone_id);
    cJSON_AddStringToObject(root, "id", str_drone_id);
    json = cJSON_PrintUnformatted(root);

    // POST JSON on URL
    response_string = post_JSON(DRONE_STATUS_URL, json);
    char answer_char[response_string.size()+1];
    strcpy(answer_char, response_string.c_str());

    // return 
    if(answer_char[0]=='Y') return 1;            // drone can go to offboard
    else if(answer_char[0]=='N') return 0;       // drone stays idle
    else return 999;                             // unknown
}

// chec if kill switch sent from server
int check_kill_server(void){

    // set response string
    std::string response_string;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    json = cJSON_PrintUnformatted(root);

    // POST JSON on URL
    response_string = post_JSON(DRONE_KILL_URL, json);
    char answer_char[response_string.size()+1];
    strcpy(answer_char, response_string.c_str());

    // return 
    return atoi(answer_char);
}
