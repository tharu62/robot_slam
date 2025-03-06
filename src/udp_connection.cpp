#include "udp_connection.h"

char udp_temp_data = 0;
struct sockaddr_in server_addr;

void udp_init(int& client, const char* ip_add, int portNum){

  client = -1;
  const char *hello = "Hello from client";
  client = socket(AF_INET, SOCK_DGRAM, 0);
  if(client < 0){
    LOG_ERROR_C("Error establishing socket...");
    exit(1);
  }

  LOG_DEBUG_C("Socket client has been created...");
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(portNum);
  server_addr.sin_addr.s_addr = inet_addr(ip_add);

  sendto(client, (const char *)hello, strlen(hello), MSG_CONFIRM, (const struct sockaddr *) &server_addr,  sizeof(server_addr));
  LOG_DEBUG_C("Connection confirmed...");
}

void udp_close(int& client){
  close(client);
  LOG_DEBUG_C("Connection terminated...");
}
