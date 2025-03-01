#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

//json libraries
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

struct Data{
  int cmd;
  int data;
};

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};


int main(int argc, char * argv[])
{
  int client = -1;
  int portNum = 80;
  const int buffsize = 70;
  char buffer[buffsize];

  struct sockaddr_in server_addr;
  
  client = socket(AF_INET, SOCK_STREAM, 0);
  if(client < 0){
    std::cout << "Error establishing socket..." << std::endl;
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

  char temp;
  std::string input_data;
  std::string str;
  json output_data;
  // loop for read and write data
  while(1){

    input_data = "";
    for(int i = 0; i < buffsize; i++){
      buffer[i] = '\0';
    }
    //std::cout << "Received: ";
    for(int i = 0; temp != '\n' && temp != '\r'; i++){
      recv(client, &temp, 1, 0);
      if(temp == '\n' || temp == '\r'){
        break;
      }
      buffer[i] = temp;
      //std::cout << buffer[i];
    }
    //std::cout << std::endl;
    temp = '\0';

    if(buffer[0] != '\0'){
      
      input_data = buffer;
      json j = json::parse(input_data);
      //std::cout << "cmd: " << j["cmd"] << std::endl;
      //std::cout << "data: " << j["data"] << std::endl;
      std::cout << "Received: " << j.dump() << std::endl;

      for(int i = 0; i < buffsize; i++){
        buffer[i] = '\0';
      }

      /** 
      switch (j["cmd"].get<int>()){
        case 1: //start
          output_data["cmd"] = 1;
          output_data["data"] = 10;
          str = output_data.dump();
          std::strncpy(buffer, str.c_str(), sizeof(buffer) - 1);
          buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
          send(client, buffer, sizeof(buffer), 0);
          break;
        case 2: //stop
          output_data["cmd"] = 2;
          output_data["data"] = 10;
          str = output_data.dump();
          std::strncpy(buffer, str.c_str(), sizeof(buffer) - 1);
          buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
          send(client, buffer, sizeof(buffer), 0);
          break;  
        default:
          break;
      }  
      */
    
    }
    
    //sleep(2);
  }

  std::cout << "Connection terminated..." << std::endl;
  close(client);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}