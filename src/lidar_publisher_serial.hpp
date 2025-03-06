#ifndef LIDAR_PUBLISHER_HPP
#define LIDAR_PUBLISHER_HPP

// librerie standard
#include <stdlib.h>
#include <string>
#include <chrono>
#include <memory>
#include <vector>

// librerie per la comunicazione seriale
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

//json libraries
#include <nlohmann/json.hpp>

#include "custom_defines.h"

#define RANGES_BUFF_SIZE 90

using namespace std::chrono_literals;

extern int client;

class Lidar_Publisher : public rclcpp::Node
{
  private:
  const char* serial_port = "/dev/ttyUSB0";

  char temp_byte;
  std::vector<char> raw_data_buffer;
  float ranges_buffer[RANGES_BUFF_SIZE];
  nlohmann::json json_data;
  sensor_msgs::msg::LaserScan msg_scan = sensor_msgs::msg::LaserScan();

  double time_a;
  double time_b;

  std::vector<int> angles;
  std::vector<float> ranges;
  std::vector<float> intensities;

  public:
  Lidar_Publisher() : Node("minimal_publisher"), time_a(0.0), time_b(0.0), ranges_buffer{0.0} {

    LOG_DEBUG_C("Opening serial port " << serial_port);
     client = open(serial_port, O_RDWR);
    if(client < 0) {
        LOG_ERROR_C("Error opening serial port: " << strerror(errno));
        exit(1);
    }
    init_serial_port(client);
    LOG_DEBUG_C("Serial port opened");

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    // timer_ = this->create_wall_timer( 1ms, [this]()->void{ this->call_back(); });
    call_back();
  }



  void call_back(){
    LOG_DEBUG_C("Callback Function called");

    while(true){
      
      if(read(client, &temp_byte, 1) < 0){
        LOG_ERROR_C("Error reading from serial port: " << strerror(errno));
        exit(1);
      }

      if(temp_byte == '{'){
        raw_data_buffer.clear();
        raw_data_buffer.push_back(temp_byte);
        continue;
      }

      if(temp_byte == '\n' && raw_data_buffer.size() > 0 && raw_data_buffer[0] == '{'){

        try{

          std::string json_str(raw_data_buffer.begin(), raw_data_buffer.end());
          json_data = nlohmann::json::parse(json_str);

        }catch(nlohmann::detail::parse_error &e){

          LOG_ERROR_C("Error parsing json: " << e.what());
          LOG_ERROR_C("Received: " << raw_data_buffer.data());
          raw_data_buffer.clear();
          intensities.clear();
          ranges.clear();
          angles.clear();
          time_a = this->now().seconds();
          continue;
        }

        //check if the json data is valid
        if(json_data["data"].size() != 4 || json_data["str"].size() != 4){
          LOG_ERROR_C("Invalid json data: " << json_data);
          raw_data_buffer.clear();
          intensities.clear();
          ranges.clear();
          angles.clear();
          time_a = this->now().seconds();
          continue;
        }

        //check if "data" or "ang" is a parameter in the json data
        if(!json_data.contains("data") || !json_data.contains("ang") || !json_data.contains("str")){
          LOG_ERROR_C("Invalid json data: " << json_data);
          raw_data_buffer.clear();
          intensities.clear();
          ranges.clear();
          angles.clear();
          time_a = this->now().seconds();
          continue;
        }

        // LOG_DEBUG_C("Received: " << json_data);
        angles.push_back(json_data["ang"]);

        ranges.push_back(json_data["data"][0].get<float>() * 0.001);
        ranges.push_back(json_data["data"][1].get<float>() * 0.001);
        ranges.push_back(json_data["data"][2].get<float>() * 0.001);
        ranges.push_back(json_data["data"][3].get<float>() * 0.001);
        
        intensities.push_back(json_data["str"][0].get<float>() * 0.001);
        intensities.push_back(json_data["str"][1].get<float>() * 0.001);
        intensities.push_back(json_data["str"][2].get<float>() * 0.001);
        intensities.push_back(json_data["str"][3].get<float>() * 0.001);

        
        if(json_data["ang"] == 89){
          

          time_a = this->now().seconds();
          msg_scan.header.stamp = this->now();
          msg_scan.header.frame_id = "laser_frame";
          msg_scan.angle_min = 0.0;
          msg_scan.angle_max =2*M_PI;
          msg_scan.angle_increment = M_PI / 180.0;
          msg_scan.time_increment = (time_b - time_a) / 360.0;
          msg_scan.scan_time = (time_b - time_a);
          msg_scan.range_min = 0.0;
          msg_scan.range_max = 100.0;
          msg_scan.ranges = ranges;
          msg_scan.intensities = intensities;
          time_b = time_a;
          
          publisher_->publish(msg_scan);

          ranges.clear();
          angles.clear();
          raw_data_buffer.clear();
          continue;
        }
        
        raw_data_buffer.clear();
        continue;
      }

      raw_data_buffer.push_back(temp_byte);
    }
  }
  
  void init_serial_port(int &tty_fd, int BAUDRATE=B460800) {

    struct termios tty_opt;
    memset(&tty_opt, 0, sizeof(tty_opt));
    tty_opt.c_cflag = CS8 | CLOCAL | CREAD; // CS8: 8n1, CLOCAL: local connection, no modem contol, CREAD: enable receiving characters
    tty_opt.c_iflag = 0;
    tty_opt.c_oflag = 0;
    tty_opt.c_lflag = 0;     // non-canonical mode
    tty_opt.c_cc[VMIN] = 1;  // blocking read until 1 character arrives
    tty_opt.c_cc[VTIME] = 0; // inter-character timer unused
    cfsetospeed(&tty_opt, BAUDRATE);
    cfsetispeed(&tty_opt, BAUDRATE);
    tcsetattr(tty_fd, TCSANOW, &tty_opt);
  }

  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  
};

#endif // LIDAR_PUBLISHER_HPP