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

using namespace std::chrono_literals;

class Lidar_Publisher : public rclcpp::Node
{
public:
Lidar_Publisher()
  : Node("minimal_publisher"), count_(0)
  {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 100);
      
      timer_ = this->create_wall_timer(500ms, [this]()->void{ this->call_back(); });
    }
    
    void call_back(){
      sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();
  
      msg_scan.header.frame_id = "laser_frame";
      msg_scan.header.stamp = this->now();
      msg_scan.angle_min = -3.14159;
      msg_scan.angle_max = 3.14159; 
      msg_scan.angle_increment = 0;
      msg_scan.time_increment = 0.01;
      msg_scan.scan_time = 0;
      msg_scan.range_min = 0.001;
      msg_scan.range_max = 8.0;
      msg_scan.ranges = {1.0,2.0};
  
      this->publisher_->publish(msg_scan);
    }
    
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  size_t count_;
};

#endif // LIDAR_PUBLISHER_HPP