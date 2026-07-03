cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
)

add_executable(mtc_pose_tester src/mtc_pose_test_node.cpp)
ament_target_dependencies(mtc_pose_tester
  rclcpp moveit_ros_planning_interface tf2_geometry_msgs tf2_eigen Eigen3)
install(TARGETS mtc_pose_tester DESTINATION lib/${PROJECT_NAME})

add_executable(mtc_ik_check src/mtc_ik_check.cpp)
ament_target_dependencies(mtc_ik_check
  rclcpp moveit_core moveit_ros_planning tf2_eigen Eigen3)
install(TARGETS mtc_ik_check DESTINATION lib/${PROJECT_NAME})

# add_executable(gripper_open_close_node src/gripper_open_close_node.cpp)
# target_compile_features(gripper_open_close_node PUBLIC c_std_99 cxx_std_17)
# ament_target_dependencies(gripper_open_close_node
#   "rclcpp"
#   "moveit_task_constructor_core"
# )

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
  )

# install(PROGRAMS
#   scripts/merge_joint_commands.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
