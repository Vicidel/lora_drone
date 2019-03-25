#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "server.h"
#include "tools.h"

void* nav(void* arg);
int set_position(struct mav &mavlink);

#endif