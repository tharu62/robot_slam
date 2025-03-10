#ifndef TELEOP_TWIST_HPP
#define TELEOP_TWIST_HPP

// librerie standard
#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

// librerie per comunicazione wireless
#include "tcp_connection.h"

//json libraries
#include <nlohmann/json.hpp>

#define BUFF_SIZE_2 100

using namespace std::chrono_literals;
using json = nlohmann::json;

extern int client;
extern struct sockaddr_in server_addr;

float v_r;
float v_l;

class Teleop_Subscriber : public rclcpp::Node
{
    private:
    char buffer[BUFF_SIZE_2];
    json j;
    std::string str;
    
    public:

    Teleop_Subscriber()
        : Node("teleop_subscriber")
    {   
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Teleop_Subscriber::twist_callback, this, std::placeholders::_1));
    }

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {        
        
        v_r = set_precision(msg->linear.x + msg->angular.z);
        v_l = set_precision(msg->linear.x - msg->angular.z);
        int temp1 = v_r * 1000;
        int temp2 = v_l * 1000;
        j = json{{"v_r", temp1}, {"v_l", temp2}};
        str = j.dump();
        for(int i = 0; i <(int) str.length(); i++)
        {
            buffer[i] = str[i];
        }
        buffer[str.length()] = '\n';
        send(client, buffer, sizeof(str)+1, 0);
    }

    private:

    float set_precision(float value)
    {   
        std::string str1 = std::to_string(value);
        std::string str2;
        int pos = str1.find(".");
        for(int i = 0; i<pos+3 && i<(int)sizeof(str1); i++)
        {
            str2[i] = str1[i];
        }
        return stof(str2);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  
};

#endif // TELEOP_TWIST_HPP