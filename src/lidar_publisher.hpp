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

#define RANGES_BUFF_SIZE 20

using namespace std::chrono_literals;
using json = nlohmann::json;

struct Data{
  int cmd;
  float angle;
  float distance;
};

extern int client;

class Lidar_Publisher : public rclcpp::Node
{
  private:
  rclcpp::Time t1;
  rclcpp::Time t2;

  double first_time = 0.0;
  double last_time = 0.0;
  float first_angle = 0.0;
  float last_angle = 0.0;
  float rpm_ = 0.0;
  float distance_[RANGES_BUFF_SIZE*4];
  json j;

  public:
  sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();
  char buffer[TCP_BUFFSIZE];
  rclcpp::Time times[TCP_BUFFSIZE];

  char temp;
  std::string input_data;
  std::string str;
  json output_data;

  Lidar_Publisher()
    : Node("minimal_publisher"), distance_{0.0}
    {
      t1 = this->now();
      t2 = this->now();
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 200);
      timer_ = this->create_wall_timer( 1us, [this]()->void{ this->call_back_2(); });
    }

  void call_back(){

    // std::chrono::time_point<std::chrono::system_clock> start, end;
    // start = std::chrono::system_clock::now();

    for(int i=0; i<RANGES_BUFF_SIZE; ++i){

      distance_[i] = 0.0;
      input_data = "";
      for(int i=0; i < TCP_BUFFSIZE; i++){
        buffer[i] = '\0';
      }
      
      for(int i = 0; temp != '\n'; i++){
        recv(client, &temp, 1, 0);
        if(temp == '\n'){
          break;
        }
        buffer[i] = temp;
      }
      temp = '\0';

      if(buffer[0] != '\0'){
        
        input_data = buffer;
        j = json::parse(input_data);
        
        if(i == 0){
          first_angle = j["angle"].get<float>() * M_PI/180.0;
          first_time = this->now().seconds();
        }
        if(i == RANGES_BUFF_SIZE-1){
          last_angle = j["angle"].get<float>() * M_PI/180.0;
          last_time = this->now().seconds();
        }
        
        // last_read_data.angle = j["angle"].get<float>() * 3.14/180.0;
        distance_[i] = j["dist"].get<float>() / 1000.0;
        
      }else{
        i--;
      }
    }
    
    msg_scan.header.frame_id = "laser_frame";
    msg_scan.header.stamp = this->now();
    msg_scan.angle_min = first_angle;
    msg_scan.angle_max = last_angle; 
    msg_scan.angle_increment = M_PI / 180.0;
    // msg_scan.angle_increment = (last_angle - first_angle) / (float)RANGES_BUFF_SIZE;
    msg_scan.time_increment = (float)(last_time - first_time) / (float)RANGES_BUFF_SIZE;
    msg_scan.scan_time = (float)(last_time - first_time);
    msg_scan.range_min = 0.00;
    msg_scan.range_max = 8.00;
    msg_scan.ranges.assign(std::begin(distance_), std::end(distance_));
    msg_scan.intensities.assign(std::begin(distance_), std::end(distance_));
    
    this->publisher_->publish(msg_scan);
    // end = std::chrono::system_clock::now();
    // LOG_DEBUG_C("Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
  }
  
  void call_back_2(){

    for(int i=0; i<RANGES_BUFF_SIZE; i++){
      input_data = "";
      distance_[i] = 0.0;
      for(int i=0; i < TCP_BUFFSIZE; i++){
        buffer[i] = '\0';
      }
  
      for(int i = 0; temp != '\n'; i++){
        recv(client, &temp, 1, 0);
        if(temp == '\n'){
          break;
        }
        buffer[i] = temp;
      }
      temp = '\0';
      
      
      if(buffer[0] != '\0'){
        input_data = buffer;
        
        j = json::parse(input_data);
        // std::cout << input_data << std::endl;
        
        distance_[0+(i*4)] = j["d1"].get<float>()/1000.0;
        distance_[1+(i*4)] = j["d2"].get<float>()/1000.0;
        distance_[2+(i*4)] = j["d3"].get<float>()/1000.0;
        distance_[3+(i*4)] = j["d4"].get<float>()/1000.0;
        
        if(i == 0){
          first_angle = j["ang"].get<float>() * M_PI/180.0;
          first_time = this->now().seconds();
        }
        if(i == RANGES_BUFF_SIZE-1){
          last_angle = j["ang"].get<float>() * M_PI/180.0;
          last_time = this->now().seconds();
        }
      }

    }    

    msg_scan.header.frame_id = "laser_frame";
    msg_scan.header.stamp = this->now();
    msg_scan.angle_min = first_angle;
    msg_scan.angle_max = last_angle; 
    msg_scan.angle_increment = M_PI / 180.0;
    msg_scan.time_increment = (float)(last_time - first_time) / (float)(RANGES_BUFF_SIZE*4);
    msg_scan.scan_time = (float)(last_time - first_time);
    msg_scan.range_min = 0.00;
    msg_scan.range_max = 5.00;
    msg_scan.ranges.assign(std::begin(distance_), std::end(distance_));
    msg_scan.intensities.assign(std::begin(distance_), std::end(distance_));
    this->publisher_->publish(msg_scan);

  }

  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  
};

#endif // LIDAR_PUBLISHER_HPP