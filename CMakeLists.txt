cmake_minimum_required(VERSION 3.8)
project(three_legged_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)
install (
  DIRECTORY launch urdf config meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

add_executable(from_zero_config_to_wheeled_stance src/from_zero_config_to_wheeled_stance.cpp)
ament_target_dependencies(
  from_zero_config_to_wheeled_stance
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs
  ros2_control
  ros2_controllers
)

add_executable(wheel_driver src/wheel_driver.cpp)
ament_target_dependencies(
  wheel_driver
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs
  ros2_control
  ros2_controllers
)
install(TARGETS
  wheel_driver
  from_zero_config_to_wheeled_stance
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
