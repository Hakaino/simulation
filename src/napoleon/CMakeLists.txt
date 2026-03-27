cmake_minimum_required(VERSION 3.8)
project(napoleon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(motor_command_gate src/motor_command_gate.cpp)
ament_target_dependencies(motor_command_gate actuator_msgs rclcpp std_msgs std_srvs)

add_executable(ground_truth_odometry src/ground_truth_odometry.cpp)
ament_target_dependencies(ground_truth_odometry geometry_msgs nav_msgs rclcpp tf2_msgs tf2_ros)

add_executable(flight_controller src/flight_controller.cpp)
ament_target_dependencies(flight_controller geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs)

add_executable(takeoff_demo src/takeoff_demo.cpp)
ament_target_dependencies(takeoff_demo rclcpp std_msgs std_srvs)

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "X4" EXCLUDE
)

install(
  TARGETS
    flight_controller
    ground_truth_odometry
    motor_command_gate
    takeoff_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
