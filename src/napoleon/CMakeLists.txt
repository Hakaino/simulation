cmake_minimum_required(VERSION 3.8)
project(napoleon)

find_package(ament_cmake REQUIRED)

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
  PROGRAMS
    scripts/flight_controller.py
    scripts/ground_truth_odometry.py
    scripts/motor_command_gate.py
    scripts/takeoff_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
