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

// librerie per comunicazione wireless
#include "tcp_connection.h"

//json libraries
#include <nlohmann/json.hpp>

#define RANGES_BUFF_SIZE 2

using namespace std::chrono_literals;
using json = nlohmann::json;

extern int client;

class Odom_Publisher : public rclcpp::Node
{
  private:
  json j;

  public:
  nav_msgs::msg::Odometry msg_odom = nav_msgs::msg::Odometry();
  char buffer[TCP_BUFFSIZE];
  rclcpp::Time times[TCP_BUFFSIZE];
  int last_pos_x = 0;

  char temp;
  std::string input_data;
  std::string str;
  json output_data;

  Odom_Publisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 200);
      timer_ = this->create_wall_timer( 100ms, [this]()->void{ this->call_back(); });
    }

  void call_back(){

    // for(int i=0; i<RANGES_BUFF_SIZE; ++i){

    //   input_data = "";
    //   for(int i=0; i < TCP_BUFFSIZE; i++){
    //     buffer[i] = '\0';
    //   }
      
    //   for(int i = 0; temp != '\n'; i++){
    //     recv(client, &temp, 1, 0);
    //     if(temp == '\n'){
    //       break;
    //     }
    //     buffer[i] = temp;
    //   }
    //   temp = '\0';

    //   if(buffer[0] != '\0'){
        
    //     input_data = buffer;
    //     j = json::parse(input_data);
        
        
    //   }else{
    //     i--;
    //   }
    // }

    geometry_msgs::msg::Vector3 linear_vel;
    linear_vel.set__x(1);
    linear_vel.set__y(0);
    linear_vel.set__z(0);
    geometry_msgs::msg::Vector3 angular_vel;
    angular_vel.set__x(0);
    angular_vel.set__y(0);
    angular_vel.set__z(0);
    last_pos_x++;

    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_link";
    msg_odom.pose.pose.position.set__x(0); 
    msg_odom.pose.pose.position.set__y(0);
    msg_odom.pose.pose.position.set__z(0);
    msg_odom.twist.twist.set__linear(linear_vel);
    msg_odom.twist.twist.set__angular(angular_vel);
    this->publisher_->publish(msg_odom);

  }
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  
};

#endif // ODOM_PUBLISHER_HPP