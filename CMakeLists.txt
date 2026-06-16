cmake_minimum_required(VERSION 3.8)
project(jmu_lunabotics)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS sensor/webcam/publish_apriltag_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
