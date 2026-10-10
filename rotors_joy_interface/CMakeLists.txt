cmake_minimum_required(VERSION 3.16)
project(rotors_joy_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rotors_comm REQUIRED)

add_executable(rotors_joy_interface_crazyflie src/joy_crazyflie.cpp)
ament_target_dependencies(rotors_joy_interface_crazyflie rclcpp sensor_msgs rotors_comm)

install(TARGETS rotors_joy_interface_crazyflie DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/key_joystick.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY media DESTINATION share/${PROJECT_NAME})

ament_package()
