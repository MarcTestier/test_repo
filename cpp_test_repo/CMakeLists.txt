cmake_minimum_required(VERSION 3.5)
project(cpp_test_repo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp std_msgs sensor_msgs)

add_executable(cpp_test_param src/cpp_test_param.cpp)
ament_target_dependencies(cpp_test_param rclcpp std_msgs sensor_msgs)

add_executable(cpp_test_urg_node_param src/cpp_test_urg_node_param.cpp)
ament_target_dependencies(cpp_test_urg_node_param rclcpp std_msgs sensor_msgs)

add_executable(parameter_events_async src/parameter_events_async.cpp)
ament_target_dependencies(parameter_events_async rclcpp std_msgs sensor_msgs)

install(TARGETS
  cpp_test
  cpp_test_param
  cpp_test_urg_node_param
  parameter_events_async
  DESTINATION lib/${PROJECT_NAME})

ament_package()
