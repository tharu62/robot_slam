#include <chrono>
#include <memory>
#include <string>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
  float angle;
  float rpm;
  float distance;
};

Data last_read_data;
float last_data_time = 0;
float last_data_angle = 0;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */
class MinimalPublisher : public rclcpp::Node
{
public:
  int val = 62;
  MinimalPublisher( ) : Node("minimal_publisher")
  {
    //publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 20);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }
  /*
  void publish_(){
    sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();
    msg_scan.header.frame_id = "laser_frame";
    msg_scan.header.stamp = this->now();
    msg_scan.angle_min = -3.14159;
    msg_scan.angle_max = 3.14159; 
    msg_scan.angle_increment = last_read_data.angle - last_data_angle;
    last_data_angle = last_read_data.angle;
    msg_scan.time_increment = 0.01;
    msg_scan.scan_time = 0;
    last_data_time = 0;
    msg_scan.range_min = 0.001;
    msg_scan.range_max = 8.0;
    msg_scan.ranges[0] = last_read_data.distance;
    this->publisher_->publish(msg_scan);
  } 
  */

  //puzzano davvero
  void pub_my_balls(){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! ";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->publisher_->publish(message);
  }

  void simple_print(){
    
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  int client = -1;
  int portNum = 80;
  const int buffsize = 64;
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
  
  auto pub_node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(pub_node);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());

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
    for(int i = 0; temp != '\n'; i++){
      recv(client, &temp, 1, 0);
      if(temp == '\n'){
        break;
      }
      buffer[i] = temp;
      //std::cout << buffer[i];
    }
    //std::cout << std::endl;
    temp = '\0';
    
    if(buffer[0] != '\0'){

      input_data = buffer;

      if(sizeof(input_data) > 1){
        
      json j = json::parse(input_data);
      //std::cout << "cmd: " << j["cmd"] << std::endl;
      //std::cout << "data: " << j["data"] << std::endl;
      //std::cout << "Received2: " << j.dump() << std::endl;

      for(int i = 0; i < buffsize; i++){
          buffer[i] = '\0';
      }
      
      last_read_data.cmd = j["cmd"].get<int>();
      last_read_data.angle = j["angle"].get<float>();
      last_read_data.rpm = j["rpm"].get<float>();
      last_read_data.distance = j["dist"].get<float>();
      
      std::cout << pub_node->val << std::endl;

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
        
    }
    sleep(1);
  }

  std::cout << "Connection terminated..." << std::endl;
  close(client);

  rclcpp::shutdown();
  return 0;
}