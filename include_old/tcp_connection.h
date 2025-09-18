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

/**
 * @brief This function creates a TCP connection with a server.
 * @param [client] It's the socket that will be created in this function.
 * @param [ip_add] It's the ip address of the server to connect with.  
 * @param [portNum] it's the port number of the server to connect with.
 */
void tcp_init(int& client, const char* ip_add="192.168.1.185", int portNum=80);

/**
 * @brief This function will close a given TCP connection.
 * @param [client] It's the TCP socket that will be closed.
 */
void tcp_close(int& client);

/**
 * @brief unused...
 * @param [client] ...
 * @param [buffer] ...
 */
void read_data(int& client, char *buffer);

#endif //TCP_CONNECTION_H