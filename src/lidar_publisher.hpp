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

class MinimalPublisher : public rclcpp::Node
{
public:
  int val = 62;
  MinimalPublisher( ) : Node("minimal_publisher")
  {
    //publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 20);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 200);
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

private:
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif // LIDAR_PUBLISHER_HPP