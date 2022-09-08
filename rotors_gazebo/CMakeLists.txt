cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo)

add_definitions(-std=c++11)

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)

if(${gazebo_VERSION_MAJOR} GREATER 2)
    message(STATUS "Building iris.sdf.")

    set(enable_mavlink_interface "true")
    set(enable_ground_truth "false")
    set(enable_logging "false")
    set(enable_camera "false")
    set(enable_wind "false")
    set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
    set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")

    # Creates shell commands to generate .sdf file
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
      COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
      DEPENDS ${rotors_description_dir}/urdf/iris.xacro
      DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
      DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
      )
    add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)
else()
  message(STATUS "Gazebo version is less than 3, cannot generate iris.sdf (gz tool not available).")
endif()

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building waypoint publisher or hovering example, returning now.")
  return()
endif()


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  rospy
  trajectory_msgs
)

find_package(
  Eigen REQUIRED
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   mav_msgs
   nav_msgs
   trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES spline_trajectory_generator
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs gazebo_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(spline_trajectory_generator
  src/library/spline_trajectory_generator.cpp
)

add_executable(waypoint_publisher src/nodes/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher_file src/nodes/waypoint_publisher_file.cpp)
target_link_libraries(waypoint_publisher_file ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher_file ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example src/nodes/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example_spline src/nodes/hovering_example_spline.cpp)
target_link_libraries(hovering_example_spline spline_trajectory_generator ${catkin_LIBRARIES})
add_dependencies(hovering_example_spline ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example_spline_swarm src/nodes/hovering_example_spline_swarm.cpp)
target_link_libraries(hovering_example_spline_swarm spline_trajectory_generator ${catkin_LIBRARIES})
add_dependencies(hovering_example_spline_swarm ${catkin_EXPORTED_TARGETS})

add_executable(quaternion_to_rpy src/nodes/quaternion_to_rpy.cpp)
target_link_libraries(quaternion_to_rpy ${catkin_LIBRARIES})
add_dependencies(quaternion_to_rpy ${catkin_EXPORTED_TARGETS})

add_executable(linear1_1 src/nodes/linear1_1.cpp)
target_link_libraries(linear1_1 ${catkin_LIBRARIES})
add_dependencies(linear1_1 ${catkin_EXPORTED_TARGETS})

add_executable(lee_semicirc src/nodes/lee_semicirc.cpp)
target_link_libraries(lee_semicirc ${catkin_LIBRARIES})
add_dependencies(lee_semicirc ${catkin_EXPORTED_TARGETS})

add_executable(lee_linear1_1_polynomial src/nodes/lee_linear1_1_polynomial.cpp)
target_link_libraries(lee_linear1_1_polynomial ${catkin_LIBRARIES})
add_dependencies(lee_linear1_1_polynomial ${catkin_EXPORTED_TARGETS})

add_executable(lee_semicirc_polynomial src/nodes/lee_semicirc_polynomial.cpp)
target_link_libraries(lee_semicirc_polynomial ${catkin_LIBRARIES})
add_dependencies(lee_semicirc_polynomial ${catkin_EXPORTED_TARGETS})

add_executable(swarm_lee_linear1_1 src/nodes/swarm_lee_linear1_1.cpp)
target_link_libraries(swarm_lee_linear1_1 ${catkin_LIBRARIES})
add_dependencies(swarm_lee_linear1_1 ${catkin_EXPORTED_TARGETS})

add_executable(swarm_lee_semicirc src/nodes/swarm_lee_semicirc.cpp)
target_link_libraries(swarm_lee_semicirc ${catkin_LIBRARIES})
add_dependencies(swarm_lee_semicirc ${catkin_EXPORTED_TARGETS})

add_executable(hovering_lee src/nodes/hovering_lee.cpp)
target_link_libraries(hovering_lee ${catkin_LIBRARIES})
add_dependencies(hovering_lee ${catkin_EXPORTED_TARGETS})

add_executable(swarm_lee_linear1_0 src/nodes/swarm_lee_linear1_0.cpp)
target_link_libraries(swarm_lee_linear1_0 ${catkin_LIBRARIES})
add_dependencies(swarm_lee_linear1_0 ${catkin_EXPORTED_TARGETS})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS waypoint_publisher waypoint_publisher_file hovering_example
  quaternion_to_rpy hovering_example_spline hovering_example_spline_swarm
  hovering_lee lee_linear1_1_polynomial lee_semicirc_polynomial lee_semicirc linear1_1
  swarm_lee_linear1_0 swarm_lee_linear1_1 swarm_lee_semicirc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
