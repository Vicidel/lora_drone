#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ros/ros.h>


#define SERVER_IP "194.209.222.197"
#define SERVER_SOCKET 10005


void send_tcp();

#endif