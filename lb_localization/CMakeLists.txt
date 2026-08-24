cmake_minimum_required(VERSION 3.16)
project(lb_localization)

find_package(ament_cmake REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(tag_localizer_core src/tag_localizer_core.cpp)
target_compile_features(tag_localizer_core PUBLIC cxx_std_17)
target_include_directories(tag_localizer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tag_localizer_core tf2 yaml_cpp_vendor)
target_link_libraries(tag_localizer_core yaml-cpp)

add_executable(tag_localizer src/tag_localizer_node.cpp)
target_compile_features(tag_localizer PUBLIC cxx_std_17)
target_link_libraries(tag_localizer tag_localizer_core)
ament_target_dependencies(tag_localizer
  apriltag_msgs
  diagnostic_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(
  TARGETS tag_localizer_core tag_localizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

# Headers are also installed conventionally for downstream unit-test or tooling use.
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(tag_localizer_core_test test/test_tag_localizer_core.cpp)
  if(TARGET tag_localizer_core_test)
    target_link_libraries(tag_localizer_core_test tag_localizer_core)
    target_compile_definitions(
      tag_localizer_core_test
      PRIVATE TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()
  ament_add_pytest_test(phase42_localizer_contract test/test_phase42_localizer_contract.py)
endif()

ament_export_include_directories(include)
ament_export_dependencies(tf2 yaml_cpp_vendor)
ament_package()
