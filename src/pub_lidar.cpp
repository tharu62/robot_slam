#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>
#include <signal.h>

//librerie per ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "lidar_publisher.hpp"
#include "tcp_connection.h"
#include "udp_connection.h"

using namespace std::chrono_literals;
using json = nlohmann::json;

/**
 * @brief Signal handler for SIGINT
 * @details This function is called when the program receives a SIGINT signal (CONTROL-C)
 */
void sigint_handler(int sig)
{
  LOG_ERROR_C("Caught signal " << sig);
  tcp_close(client);
  rclcpp::shutdown();
  exit(0);
}

int client;

int main(int argc, char * argv[])
{ 
  //signal handler
  signal(SIGINT, sigint_handler);
  
  rclcpp::init(argc, argv);
  //tcp_init(client);
  udp_init(client); 
  auto pub_node = std::make_shared<Lidar_Publisher>();
  rclcpp::spin(pub_node);
  
  tcp_close(client);
  rclcpp::shutdown();
  return 0;
}