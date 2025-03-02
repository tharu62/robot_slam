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
      timer_ = this->create_wall_timer(10ms, 
      [this]()->void{ 
        this->call_back();
      });
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
        last_read_data.angle = j["angle"].get<float>();
        last_read_data.rpm = j["rpm"].get<float>();
        last_read_data.distance = j["dist"].get<float>();
        
        msg_scan.header.frame_id = "laser_frame";
        msg_scan.header.stamp = this->now();
        msg_scan.angle_min = -3.14159;
        msg_scan.angle_max = 3.14159; 
        msg_scan.angle_increment = last_read_data.angle - last_data_angle;
        last_data_angle = last_read_data.angle;
        time_ = this->now().seconds();
        msg_scan.time_increment = (float) (time_ - last_data_time);
        last_data_time = time_;
        msg_scan.scan_time = time_;
        msg_scan.range_min = 0.001f;
        msg_scan.range_max = 8.0f;
        msg_scan.ranges = {last_read_data.distance};
        this->publisher_->publish(msg_scan);
      }
    }
      
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t count_;
};

#endif // LIDAR_PUBLISHER_HPP