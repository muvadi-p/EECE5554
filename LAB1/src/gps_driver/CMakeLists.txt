cmake_minimum_required(VERSION 3.8)
project(gps_driver)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpsMsg.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME driver
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
