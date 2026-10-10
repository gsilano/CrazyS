cmake_minimum_required(VERSION 3.16)
project(rotors_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

# Lets Gazebo Sim resolve the package:// URIs of the meshes.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
