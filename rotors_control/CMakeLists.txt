cmake_minimum_required(VERSION 3.16)
project(rotors_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rotors_comm REQUIRED)

# Control library: plain C++ and Eigen, no ROS.
add_library(rotors_control_lib SHARED
  src/library/crazyflie_complementary_filter.cpp
  src/library/crazyflie_onboard_controller.cpp
  src/library/internal_model_controller.cpp
  src/library/lee_geometric_controller.cpp
  src/library/position_controller.cpp
  src/library/roll_pitch_yawrate_thrust_crazyflie.cpp
  src/library/sensfusion6.cpp
)
target_include_directories(rotors_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rotors_control_lib PUBLIC Eigen3::Eigen)

# ROS 2 nodes
set(node_dependencies rclcpp nav_msgs sensor_msgs trajectory_msgs actuator_msgs rotors_comm)
foreach(node position_controller_node lee_geometric_controller_node
             roll_pitch_yawrate_thrust_crazyflie_node)
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} rotors_control_lib)
  ament_target_dependencies(${node} ${node_dependencies})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS rotors_control_lib EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers rotors_control_lib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
