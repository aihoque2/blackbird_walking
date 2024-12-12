cmake_minimum_required(VERSION 3.8)
project(blackbird_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(gz-plugin2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# directory install
install(DIRECTORY
  include/
  DESTINATION include
)

# libraries
add_library(BlackbirdPosePublisher SHARED
include/PosePublisher.h
src/PosePublisher.cpp
)

target_include_directories(BlackbirdPosePublisher PRIVATE include)

ament_target_dependencies(BlackbirdPosePublisher
  gz-sim8
  gz-plugin2
  rclcpp
  geometry_msgs
  std_msgs
)

#executables
add_executable(effort_example_cpp src/effort_example.cpp)
ament_target_dependencies(effort_example_cpp rclcpp std_msgs)

add_executable(reverse_effort_cpp src/reverse_effort.cpp)
ament_target_dependencies(reverse_effort_cpp rclcpp std_msgs)



install(
  DIRECTORY 
  launch rviz urdf meshes config src
  
  DESTINATION 
  share/${PROJECT_NAME}/
)


# C++ install
install(TARGETS
  BlackbirdPosePublisher
  effort_example_cpp
  reverse_effort_cpp
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_export_include_directories(
  include
)

ament_package()
