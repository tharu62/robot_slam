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
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_broadcaster.h>
#include "my_robot_interfaces/msg/dif_drive.hpp"

#define BUFF_SIZE_1 50
#define RADIUS 0.04
#define WHEELBASE 0.3
#define REDUCTION_RATIO 5940.0
#define MAX_STEPS_RIGHT 1365
#define MAX_STEPS_LEFT 1365

using namespace std::chrono_literals;

extern int client;

/**
 * @brief This is a class that takes as input the values of revolution [N_left] [N_right] of the encoder 
 * on the wheels, the delta time [delta_time] necessary to make the revolution and produces as output 
 * the odometry message of a differential drive robot with the specified parameters defined above.
 * @note [Input] is receved by ros2 topic <dif_drive>. 
 * @note [Output] is published on the /odom topic.
 */
class Odom_Publisher : public rclcpp::Node
{
  private:
  double new_x = 0.0;
  float angle = 0.0;
  float D_left = 0.0;
  float D_right = 0.0;
  float D_avg = 0.0;
  float x = 0.0, y = 0.0, theta = 0.0;
  float delta_x = 0.0;
  float delta_y = 0.0;
  float delta_theta = 0.0;
  float delta_time = 0.1;
  float v_r;
  float v_l;
  
  public:
  nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();

  Odom_Publisher()
    : Node("odom_publisher")
    {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      publisher_  = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      subscription_ = this->create_subscription<my_robot_interfaces::msg::DifDrive>("dif_drive", 10, std::bind(&Odom_Publisher::call_back, this, std::placeholders::_1));

    }

  void call_back(my_robot_interfaces::msg::DifDrive::SharedPtr msg){
  // void call_back(){

    // read wheel steps from the topic
    delta_time = msg->dt;

    // Create and populate the Odometry message
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Oodometry calculation
    D_left = msg->left_wheel_steps*2*M_PI*RADIUS/MAX_STEPS_LEFT;
    D_right = msg->right_wheel_steps*2*M_PI*RADIUS/MAX_STEPS_RIGHT;
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

    // Create and populate the TransformStamped message for base_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    // odometry message
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;    

    // Broadcast the transform for base_link
    tf_broadcaster_->sendTransform(transform);
  }
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<my_robot_interfaces::msg::DifDrive>::SharedPtr subscription_;
  
};

#endif // ODOM_PUBLISHER_HPP