cmake_minimum_required(VERSION 3.16)
project(lb_sensors)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(terrain_hazard_core src/terrain_hazard_core.cpp)
target_compile_features(terrain_hazard_core PUBLIC cxx_std_17)
target_include_directories(terrain_hazard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(terrain_hazard_node src/terrain_hazard_node.cpp)
target_compile_features(terrain_hazard_node PUBLIC cxx_std_17)
target_link_libraries(terrain_hazard_node terrain_hazard_core)
ament_target_dependencies(terrain_hazard_node
  diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

install(TARGETS terrain_hazard_core terrain_hazard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(phase41_webcam_contract test/test_phase41_webcam_contract.py)
  ament_add_gtest(terrain_hazard_core_test test/test_terrain_hazard_core.cpp)
  if(TARGET terrain_hazard_core_test)
    target_link_libraries(terrain_hazard_core_test terrain_hazard_core)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
