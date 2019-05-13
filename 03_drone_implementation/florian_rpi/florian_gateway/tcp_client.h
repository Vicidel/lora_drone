#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include "serial.h"
#include "tools.h"

#define SERVER_IP "178.193.106.50"
#define SERVER_SOCKET 10005

void* send_tcp(void* arg);

#endif