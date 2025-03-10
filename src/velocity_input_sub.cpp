#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "teleop_twist_sub.hpp"
#include "tcp_connection.h"

int client;

void sigint_handler(int sig)
{
  LOG_ERROR_C("Caught signal " << sig);
  tcp_close(client);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char * argv[])
{
  //signal handler
  signal(SIGINT, sigint_handler);

  rclcpp::init(argc, argv);
  tcp_init(client);

  rclcpp::spin(std::make_shared<Teleop_Subscriber>());

  tcp_close(client);
  rclcpp::shutdown();
  return 0;
}