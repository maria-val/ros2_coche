cmake_minimum_required(VERSION 3.8)
project(c4_buscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buscan_msgs REQUIRED)

add_executable(buscan_pub src/buscan.cpp)
ament_target_dependencies(buscan_pub rclcpp buscan_msgs)

add_executable(can_parser src/can_parser.cpp)
ament_target_dependencies(can_parser rclcpp buscan_msgs)

target_link_libraries(buscan_pub canlib)

include_directories(include)

install(TARGETS
  buscan_pub
  can_parser
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
