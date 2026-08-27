cmake_minimum_required(VERSION 3.8)
project(tb3_bringup)

find_package(ament_cmake REQUIRED)

# Data-only package: no targets, no source. Every directory listed here must
# exist in the source tree — CMake's install(DIRECTORY) fails the build when a
# listed directory is missing, and git does not track empty directories.
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
