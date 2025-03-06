#ifndef UDP_CONNECTION_H
#define UDP_CONNECTION_H

//librerie per socket programming
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>

#include "custom_defines.h"

#define UDP_BUFFSIZE 64

extern char udp_temp_data;

void udp_init(int& client, const char* ip_add="192.168.1.162", int portNum=80);

void udp_close(int& client);

#endif //UDP_CONNECTION_H