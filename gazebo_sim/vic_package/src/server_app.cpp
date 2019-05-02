#include "server_app.h"



size_t writeFunction(void *ptr, size_t size, size_t nmemb, std::string* data) {
    data->append((char*) ptr, size * nmemb);
    return size * nmemb;
}


// POST the drone GPS coordinates
std::string send_GPS(Vector3f position, double time, char* payload){

    // set response string
    std::string response_string;

    // create curl object
    CURL *curl = NULL;
    CURLcode res = CURLE_FAILED_INIT; 
    struct curl_slist *headers = NULL;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_pos_x[10]; sprintf(str_pos_x, "%f", position(0));
    char str_pos_y[10]; sprintf(str_pos_y, "%f", position(1));
    char str_pos_z[10]; sprintf(str_pos_z, "%f", position(2));
    char str_time[30]; sprintf(str_time, "%f", time);
    cJSON_AddStringToObject(root, "pos_x", str_pos_x);
    cJSON_AddStringToObject(root, "pos_y", str_pos_y);
    cJSON_AddStringToObject(root, "pos_z", str_pos_z);
    cJSON_AddStringToObject(root, "timestamp", str_time);
    cJSON_AddStringToObject(root, "payload", payload);
    json = cJSON_PrintUnformatted(root);
    //std::cout << "String sent:" << json << std::endl;

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
        curl_easy_setopt(curl, CURLOPT_URL, DRONE_POST_GPS_URL);
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

    std::cout << "POST answer: " << response_string << std::endl;

    return response_string;
}

// POST the drone GPS coordinates and drone number
std::string send_GPS_drone3(Vector3f position, double time, char* payload, int no_drone){

    // set response string
    std::string response_string;

    // create curl object
    CURL *curl = NULL;
    CURLcode res = CURLE_FAILED_INIT; 
    struct curl_slist *headers = NULL;

    // create JSON to send
    cJSON *root = NULL;
    char *json = NULL;
    root = cJSON_CreateObject();

    // fils json
    char str_pos_x[10];    sprintf(str_pos_x, "%f", position(0));
    char str_pos_y[10];    sprintf(str_pos_y, "%f", position(1));
    char str_pos_z[10];    sprintf(str_pos_z, "%f", position(2));
    char str_time[30];     sprintf(str_time, "%f", time);
    char str_no_drone[10]; sprintf(str_no_drone, "%d", no_drone);
    cJSON_AddStringToObject(root, "pos_x", str_pos_x);
    cJSON_AddStringToObject(root, "pos_y", str_pos_y);
    cJSON_AddStringToObject(root, "pos_z", str_pos_z);
    cJSON_AddStringToObject(root, "timestamp", str_time);
    cJSON_AddStringToObject(root, "payload", payload);
    cJSON_AddStringToObject(root, "no_drone", str_no_drone)
    json = cJSON_PrintUnformatted(root);
    //std::cout << "String sent:" << json << std::endl;

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
        curl_easy_setopt(curl, CURLOPT_URL, DRONE_POST_GPS_URL);
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

    std::cout << "POST answer: " << response_string << std::endl;

    return response_string;
}

