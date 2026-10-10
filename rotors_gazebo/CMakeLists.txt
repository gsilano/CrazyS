cmake_minimum_required(VERSION 3.16)
project(rotors_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rotors_comm REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(spline_trajectory_generator SHARED src/library/spline_trajectory_generator.cpp)
target_include_directories(spline_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(spline_trajectory_generator PUBLIC Eigen3::Eigen)

set(node_dependencies rclcpp geometry_msgs nav_msgs trajectory_msgs rotors_comm)
foreach(node hovering_example hovering_example_spline waypoint_publisher
             waypoint_publisher_file quaternion_to_rpy piecewise_trajectory_generator)
  add_executable(${node} src/nodes/${node}.cpp)
  ament_target_dependencies(${node} ${node_dependencies})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
target_link_libraries(hovering_example_spline spline_trajectory_generator)
target_link_libraries(piecewise_trajectory_generator Eigen3::Eigen yaml-cpp)

install(TARGETS spline_trajectory_generator EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
