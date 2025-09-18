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

/**
 * @brief This function creates a UDP connection with a server.
 * @param [client] It's the socket that will be created in this function.
 * @param [ip_add] It's the ip address of the server to connect with.  
 * @param [portNum] it's the port number of the server to connect with.
 */
void udp_init(int& client, const char* ip_add="192.168.1.185", int portNum=80);

/**
 * @brief This function will close a given UDP connection.
 * @param [client] It's the UDP socket that will be closed.
 */
void udp_close(int& client);

#endif //UDP_CONNECTION_H