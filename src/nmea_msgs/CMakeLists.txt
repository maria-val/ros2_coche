cmake_minimum_required(VERSION 3.8)
project(nmea_msgs)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpgga.msg"
  "msg/Gpgsa.msg"
  "msg/Gpgsv.msg"
  "msg/GpgsvSatellite.msg"
  "msg/Gprmc.msg"
  "msg/Gpgst.msg"
  "msg/Sentence.msg"
  DEPENDENCIES std_msgs
)

# Install
install(DIRECTORY "msg"
  DESTINATION "share/${PROJECT_NAME}"
)

# Export dependencies
ament_export_dependencies(std_msgs)

# Package information
ament_package()
