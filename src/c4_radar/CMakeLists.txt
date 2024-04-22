cmake_minimum_required(VERSION 3.8)
project(c4_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(radar_reader src/radar_reader.cpp)
ament_target_dependencies(radar_reader rclcpp radar_msgs)

add_executable(radar_parser src/radar_parser.cpp src/radar_ars408.cpp src/radar_srr208.cpp)
ament_target_dependencies(radar_parser rclcpp radar_msgs)

add_executable(radar_visualizer src/radar_visualization.cpp)
ament_target_dependencies(radar_visualizer rclcpp radar_msgs visualization_msgs)

include_directories(include)

install(TARGETS
  radar_reader
  radar_parser
  radar_visualizer
  DESTINATION lib/${PROJECT_NAME})

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
