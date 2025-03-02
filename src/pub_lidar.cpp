#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "lidar_publisher.hpp"
#include "tcp_connection.h"

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


int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  int client;
  tcp_init(client);
  char buffer[TCP_BUFFSIZE];
    
  auto pub_node = std::make_shared<Lidar_Publisher>();
  rclcpp::spin(pub_node);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());

  char temp;
  std::string input_data;
  std::string str;
  json output_data;
  // loop for read and write data
  while(1){

    // input_data = "";
    // for(int i = 0; i < TCP_BUFFSIZE; i++){
    //   buffer[i] = '\0';
    // }
    // //std::cout << "Received: ";
    // for(int i = 0; temp != '\n'; i++){
    //   recv(client, &temp, 1, 0);
    //   if(temp == '\n'){
    //     break;
    //   }
    //   buffer[i] = temp;
    //   //std::cout << buffer[i];
    // }
    // //std::cout << std::endl;
    // temp = '\0';
    
    // if(buffer[0] != '\0'){

    //   input_data = buffer;

    //   //std::cout << "Input Data: ";
    //   //std::cout << input_data << std::endl;

    //   pub_node->pub_my_balls();
    //   usleep(20000);

    //   // json j = json::parse(input_data);
    //   // //std::cout << "cmd: " << j["cmd"] << std::endl;
    //   // //std::cout << "data: " << j["data"] << std::endl;
    //   // std::cout << "Received2: " << j.dump() << std::endl;

    //   // for(int i = 0; i < TCP_BUFFSIZE; i++){
    //   //     buffer[i] = '\0';
    //   // }
      
    //   // last_read_data.cmd = j["cmd"].get<int>();
    //   // last_read_data.angle = j["angle"].get<float>();
    //   // last_read_data.rpm = j["rpm"].get<float>();
    //   // last_read_data.distance = j["dist"].get<float>();
      
    //   // std::cout << pub_node->val << std::endl;

    //   /** 
    //    switch (j["cmd"].get<int>()){
    //     case 1: //start
    //     output_data["cmd"] = 1;
    //     output_data["data"] = 10;
    //     str = output_data.dump();
    //     std::strncpy(buffer, str.c_str(), sizeof(buffer) - 1);
    //     buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    //     send(client, buffer, sizeof(buffer), 0);
    //     break;
    //     case 2: //stop
    //     output_data["cmd"] = 2;
    //     output_data["data"] = 10;
    //     str = output_data.dump();
    //     std::strncpy(buffer, str.c_str(), sizeof(buffer) - 1);
    //     buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    //     send(client, buffer, sizeof(buffer), 0);
    //     break;  
    //     default:
    //     break;
    //     }  
    //     */
    // }



  }

  tcp_close(client);
  rclcpp::shutdown();
  return 0;
}