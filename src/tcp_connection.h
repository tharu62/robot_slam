#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

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

#define TCP_BUFFSIZE 64

extern char tcp_temp_data;

void tcp_init(int& client, const char* ip_add="192.168.118.239", int portNum=80);

void tcp_close(int& client);

void read_data(int& client, char *buffer);

#endif //TCP_CONNECTION_H