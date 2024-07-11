cmake_minimum_required(VERSION 3.8)
project(c4_epos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(epos_msgs REQUIRED)
find_package(buscan_msgs REQUIRED)

include_directories(include)

link_directories(/usr/lib/epos)

add_executable(epos_reader src/EPOS.cpp src/epos_reader.cpp)
ament_target_dependencies(epos_reader rclcpp std_msgs epos_msgs buscan_msgs)

add_executable(mode_control src/mode_control.cpp)
ament_target_dependencies(mode_control rclcpp std_msgs epos_msgs)

#add_executable(epos_controller src/EPOS.cpp src/epos_controller.cpp)
#ament_target_dependencies(epos_controller rclcpp std_msgs epos_msgs)

#target_link_directories(epos_reader PRIVATE /usr/lib/epos)

target_link_libraries(epos_reader EposCmd)
#target_link_libraries(epos_controller EposCmd)

install(TARGETS
  epos_reader
  mode_control
  #epos_controller
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
