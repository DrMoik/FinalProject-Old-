cmake_minimum_required(VERSION 3.5)
project(web_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_web_republisher REQUIRED)
find_package(web_video_server REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
