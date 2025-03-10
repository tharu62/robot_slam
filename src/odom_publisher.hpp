#ifndef ODOM_PUBLISHER_HPP
#define ODOM_PUBLISHER_HPP

// librerie standard
#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_broadcaster.h>

// librerie per comunicazione wireless
#include "tcp_connection.h"

//json libraries
#include <nlohmann/json.hpp>

#define BUFF_SIZE 30
#define RADIUS 0.03
#define WHEELBASE 0.2

using namespace std::chrono_literals;
using json = nlohmann::json;

extern int client;

class Odom_Publisher : public rclcpp::Node
{
  private:
  json j;
  double new_x = 0.0;
  float angle = 0.0;

  float N_left = 0.0;
  float N_right = 0.0;
  float D_left = 0.0;
  float D_right = 0.0;
  float D_avg = 0.0;
  float x = 0.0, y = 0.0, theta = 0.0;
  float delta_x = 0.0;
  float delta_y = 0.0;
  float delta_theta = 0.0;
  float delta_time = 0.1;

  public:
  nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
  char buffer[BUFF_SIZE];
  char temp;
  std::string input_data;
  std::string str;

  Odom_Publisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // tf_broadcaster_left_wheel = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // tf_broadcaster_right_wheel = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      timer_ = this->create_wall_timer( 100ms, [this]()->void{ this->call_back();});
    }

  void call_back(){

    input_data = "";
    for(int i=0; i < TCP_BUFFSIZE; i++){
      buffer[i] = '\0';
    }
    for(int i=0; i<BUFF_SIZE; i++){
      for(int i = 0; temp != '\n'; i++){
        recv(client, &temp, 1, 0);
        if(temp == '\n'){
          break;
        }
        buffer[i] = temp;
      }
    }
    temp = '\0';
    if(buffer[0] != '\0'){
      input_data = buffer;
      j = json::parse(input_data);
      N_left = j["d1"];
      N_right = j["d2"];
    }

    // Create and populate the Odometry message
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    // Input data
    N_left += 0.001;
    N_right += 0.001;
    // Oodometry calculation
    D_left = N_left*2*M_PI*RADIUS;
    D_right = N_right*2*M_PI*RADIUS;
    D_avg = (D_left + D_right)/2;
    delta_theta = (D_right - D_left)/WHEELBASE;
    tf2::Quaternion q;
    q.setRPY(0,0,(theta + delta_theta));
    theta += delta_theta;
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    delta_x = D_avg*cos(theta);
    delta_y = D_avg*sin(theta);
    odom_msg.pose.pose.position.x = x + delta_x;
    odom_msg.pose.pose.position.y = y + delta_y;
    x = odom_msg.pose.pose.position.x;
    y = odom_msg.pose.pose.position.y;
    odom_msg.twist.twist.linear.x = D_avg/delta_time;
    odom_msg.twist.twist.angular.z = delta_theta/delta_time;
    // Publish the odometry message
    this->publisher_->publish(odom_msg);

    // Create and populate the TransformStamped message
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    // Here we use the same dummy data as the odometry message
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;    
    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);

    if(N_left == 0.1){
      N_left = 0;
    }
    if(N_right == 0.1){
      N_right = 0;
    }

  }
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_left_wheel;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_roght_wheel;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  
};

#endif // ODOM_PUBLISHER_HPP