#include "tcp_connection.h"

char tcp_temp_data = 0;

void tcp_init(int& client, int portNum){

  client = -1;
  struct sockaddr_in server_addr;
  client = socket(AF_INET, SOCK_STREAM, 0);
  if(client < 0){
    LOG_ERROR_C("Error establishing socket...");
    exit(1);
  }

  LOG_DEBUG_C("Socket client has been created...");
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(portNum);
  server_addr.sin_addr.s_addr = inet_addr("192.168.1.185");
  int connection = 1;

  while(connection != 0){
    LOG_DEBUG_C("Connecting to server...");
    connection = connect(client, (struct sockaddr *)&server_addr, sizeof(server_addr));
    sleep(2);
  }

  LOG_DEBUG_C("Connection confirmed...");
}

void tcp_close(int& client){
  close(client);
  LOG_DEBUG_C("Connection terminated...");
}

void read_data(int& client, char *buffer){

  for(int i = 0; i < TCP_BUFFSIZE; ++i){
    buffer[i] = '\0';
  }
  
  for(int i = 0; tcp_temp_data != '\n'; ++i){
    recv(client, &tcp_temp_data, 1, 0);
    if(tcp_temp_data == '\n'){
      break;
    }
    buffer[i] = tcp_temp_data;
  }
  tcp_temp_data = '\0';
  
}