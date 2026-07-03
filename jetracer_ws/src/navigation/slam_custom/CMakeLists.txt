cmake_minimum_required(VERSION 3.5)
project(slam_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  params
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
