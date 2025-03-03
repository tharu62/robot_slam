#ifndef LIDAR_PUBLISHER_HPP
#define LIDAR_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

extern Data last_read_data;
extern double last_data_time;
extern float last_data_angle;
extern int client;

class Lidar_Publisher : public rclcpp::Node
{
  public:
  sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();
  char buffer[TCP_BUFFSIZE];
  char temp;
  double time_;
  std::string input_data;
  std::string str;
  json output_data;

  Lidar_Publisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 100);
      timer_ = this->create_wall_timer( 10ms, [this]()->void{ this->call_back(); });
    }

  void call_back(){
    
    input_data = "";
    for(int i = 0; i < TCP_BUFFSIZE; i++){
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
      
      for(int i = 0; i < TCP_BUFFSIZE; i++){
        buffer[i] = '\0';
      }
      
      json j = json::parse(input_data);
      last_read_data.angle = j["angle"].get<float>() * 3.14/180.0;
      last_read_data.rpm = j["rpm"].get<float>();
      last_read_data.distance = j["dist"].get<float>();
      
      msg_scan.header.frame_id = "laser_frame";
      msg_scan.header.stamp = this->now();

      msg_scan.angle_min = last_read_data.angle;
      msg_scan.angle_max = last_data_angle; 

      // msg_scan.angle_increment = last_read_data.angle - last_data_angle;
      msg_scan.angle_increment = 1.1f;

      last_data_angle = last_read_data.angle;
      time_ = this->now().seconds();

      msg_scan.time_increment = 0.0;

      last_data_time = time_;
      msg_scan.scan_time = 0.1;

      msg_scan.range_min = 0.001f;
      msg_scan.range_max = 10.0f;
      msg_scan.ranges = {1.0f, 1.1f};
      msg_scan.intensities = {1.0f, 1.0f};

      this->publisher_->publish(msg_scan);
    }
  }
      
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t count_;
};

#endif // LIDAR_PUBLISHER_HPP