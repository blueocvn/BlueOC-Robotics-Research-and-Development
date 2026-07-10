cmake_minimum_required(VERSION 3.5)
project(jetracer_bringup)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# dock_pose_publisher: bridges apriltag TF -> detected_dock_pose for opennav_docking
install(PROGRAMS
  scripts/dock_pose_publisher.py
  scripts/jetracer_docker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
