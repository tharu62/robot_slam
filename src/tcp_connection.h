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

#define TCP_BUFFSIZE 64

void tcp_init(int& client, int portNum=80){

    client = -1;
    struct sockaddr_in server_addr;
    client = socket(AF_INET, SOCK_STREAM, 0);
    if(client < 0){
        std::cerr << "Error establishing socket..." << std::endl;
        exit(1);
    }

    std::cout << "Socket client has been created..." << std::endl;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(portNum);
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.185");
    int connection = 1;

    while(connection != 0){
        std::cout << "Connecting to server..." << std::endl;
        connection = connect(client, (struct sockaddr *)&server_addr, sizeof(server_addr));
        sleep(2);
    }

    std::cout << "Connection confirmed..." << std::endl;
}

void tcp_close(int& client){
    close(client);
    std::cout << "Connection terminated..." << std::endl;
}

char temp;

char * read_data(char* buffer){
      for(int i = 0; i < TCP_BUFFSIZE; i++){
        buffer[i] = '\0';
      }
      for(int i = 0; temp != '\n'; i++){
        recv(client, &temp, 1, 0);
        if(temp == '\n'){
          break;
        }
        buffer[i] = temp;
      }
      temp = '\0';
      return buffer;
}
#endif //TCP_CONNECTION_H