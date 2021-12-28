cmake_minimum_required(VERSION 3.0.2)
project(rotors_joy_interface VERSION 7.0.1)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  sensor_msgs
  trajectory_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs sensor_msgs trajectory_msgs
  LIBRARIES
)

add_executable(rotors_joy_interface src/joy.cpp)
add_dependencies(rotors_joy_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_joy_interface ${catkin_LIBRARIES})

add_executable(rotors_joy_interface_crazyflie src/joy_crazyflie.cpp)
add_dependencies(rotors_joy_interface_crazyflie ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_joy_interface_crazyflie ${catkin_LIBRARIES})

install(TARGETS rotors_joy_interface rotors_joy_interface_crazyflie
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
