cmake_minimum_required(VERSION 3.8)
project(robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# add include directory
INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
)

# adding nodes installation
install(TARGETS
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