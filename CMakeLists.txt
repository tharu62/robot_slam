cmake_minimum_required(VERSION 3.8)
project(robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# add_executable(talker_lidar src/pub_lidar.cpp src/tcp_connection.cpp src/udp_connection.cpp)
# ament_target_dependencies(talker_lidar rclcpp std_msgs sensor_msgs geometry_msgs)

# add_executable(talker_odom src/pub_odom.cpp src/tcp_connection.cpp)
add_executable(talker_odom src/pub_odom.cpp)
ament_target_dependencies(talker_odom rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs my_robot_interfaces)

add_executable(odom_tf2 src/odom_tf2_broadcaster.cpp)
ament_target_dependencies(odom_tf2 rclcpp tf2 tf2_ros geometry_msgs)

# add_executable(lidar_tf2 src/lidar_tf2_broadcaster.cpp)
# ament_target_dependencies(lidar_tf2 rclcpp tf2 tf2_ros geometry_msgs)

# unused node listener_keys
# add_executable(listener_keys src/sub_velocity_input.cpp src/tcp_connection.cpp)
# ament_target_dependencies(listener_keys rclcpp geometry_msgs)

# add include directory
INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
)

# adding nodes installation
install(TARGETS
  # talker_lidar
  talker_odom
  odom_tf2
  # lidar_tf2
  # listener_keys
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# adding launch file installation
install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()