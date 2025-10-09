#ifndef LIDAR_PUBLISHER_HPP
#define LIDAR_PUBLISHER_HPP

// librerie standard
#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// librerie per comunicazione wireless

#include "tcp_connection.h"

//json libraries
#include <nlohmann/json.hpp>

#define BUFF_SIZE 20

using namespace std::chrono_literals;
using json = nlohmann::json;

extern int client;
extern struct sockaddr_in server_addr;


/**
 * @brief This is a class that takes as input the values of a distance and angle read from a lidar sensor
 * and return as output the scan value in the correct format. 
 * @note [Input] is received by UDP connection as string with the format: "a<angle>d<distance>", where <angle> and 
 * <distance> are int values. 
 * @note [Output] is published on the /laser_scan topic.
 */
class Lidar_Publisher : public rclcpp::Node
{
  private:
  double first_time = 0.0;
  double last_time = 0.0;
  unsigned int len;
  std::vector<float> vec;
  std::string angle_;
  std::string dist_;
  std::string test;

  public:
    sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();
    char buffer[BUFF_SIZE];

  Lidar_Publisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 20);
      // timer_ = this->create_wall_timer( 0ms, [this]()->void{ this->call_back(); });
    }

    
  void call_back(){
    for(int i=0; i < 20; i++){
      buffer[i] = '\0';
    }
    recvfrom(client, buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &server_addr, &len);
    first_time = this->now().seconds();

    int d_indx=0;
    for(int i=0; i<(int)sizeof(buffer); ++i){
      if(buffer[i] == 'd') d_indx = i;
    }
    test = std::string(buffer);
    angle_ = test.substr(1,d_indx-1);
    dist_ = test.substr(d_indx+1, sizeof(buffer));
    vec.push_back((float) stof(dist_)/1000.0);
    msg_scan.angle_min = (float) stof(angle_)*M_PI / 180.0;
    
    for(int i=0; i < 20; i++){
      buffer[i] = '\0';
    }
    recvfrom(client, buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &server_addr, &len);
    last_time = this->now().seconds();
    
    d_indx=0;
    for(int i=0; i<(int)sizeof(buffer); ++i){
      if(buffer[i] == 'd') d_indx = i;
    }
    test = std::string(buffer);
    angle_ = test.substr(1,d_indx-1);
    dist_ = test.substr(d_indx+1, sizeof(buffer));
    vec.push_back((float) stof(dist_)/1000.0);
    msg_scan.angle_max = (float) stof(angle_)*M_PI / 180.0;

    msg_scan.header.frame_id = "laser_frame";
    msg_scan.header.stamp = this->now();
    msg_scan.angle_increment = (float) (msg_scan.angle_max - msg_scan.angle_min);
    msg_scan.time_increment = (float) (last_time - first_time);
    msg_scan.scan_time = (float) (last_time - first_time);
    msg_scan.range_min = 0.00;
    msg_scan.range_max = 3.00;
    msg_scan.ranges = vec;
    msg_scan.intensities = vec;
    this->publisher_->publish(msg_scan); 
    vec.clear();
  }
  
  // void call_back_tcp(){
  // for(int i=0; i<BUFF_SIZE; ++i){
  //   distance_[i] = 0.0;
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
  //     if(i == 0){
  //       first_angle = j["angle"].get<float>() * M_PI/180.0;
  //       first_time = this->now().seconds();
  //     }
  //     if(i == RANGES_BUFF_SIZE-1){
  //       last_angle = j["angle"].get<float>() * M_PI/180.0;
  //       last_time = this->now().seconds();
  //     }
  //     distance_[i] = j["dist"].get<float>() / 1000.0;      
  //   }else{
  //     i--;
  //   }
  // }  
  // msg_scan.header.frame_id = "laser_frame";
  // msg_scan.header.stamp = this->now();
  // msg_scan.angle_min = first_angle;
  // msg_scan.angle_max = last_angle; 
  // msg_scan.angle_increment = M_PI / 180.0;
  // msg_scan.time_increment = (float)(last_time - first_time) / (float)BUFF_SIZE;
  // msg_scan.scan_time = (float)(last_time - first_time);
  // msg_scan.range_min = 0.00;
  // msg_scan.range_max = 8.00;
  // msg_scan.ranges.assign(std::begin(distance_), std::end(distance_));
  // msg_scan.intensities.assign(std::begin(distance_), std::end(distance_)); 
  // this->publisher_->publish(msg_scan);
  // }
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  
};

#endif // LIDAR_PUBLISHER_HPP