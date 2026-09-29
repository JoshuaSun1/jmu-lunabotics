cmake_minimum_required(VERSION 3.16)
project(lb_hardware)

find_package(ament_cmake REQUIRED)

add_library(lb_drive_transport SHARED
  src/mock_drive_transport.cpp
  src/real_drive_transport.cpp
)
target_compile_features(lb_drive_transport PUBLIC cxx_std_17)
target_include_directories(lb_drive_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(lb_drive_transport PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS lb_drive_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The transport library is independently testable on the current structural host.
# Build the ROS plugin only where the target ros2_control dependencies are available.
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)
if(hardware_interface_FOUND AND pluginlib_FOUND AND rclcpp_FOUND AND rclcpp_lifecycle_FOUND)
  add_library(lb_hardware_plugin SHARED src/lunabot_drive_hardware.cpp)
  target_compile_features(lb_hardware_plugin PUBLIC cxx_std_17)
  target_include_directories(lb_hardware_plugin
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(lb_hardware_plugin PRIVATE lb_drive_transport)
  ament_target_dependencies(
    lb_hardware_plugin
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lb_hardware_plugin PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  pluginlib_export_plugin_description_file(hardware_interface lb_hardware_plugins.xml)
  install(
    TARGETS lb_hardware_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
else()
  message(STATUS "Skipping lb_hardware ros2_control plugin: target ROS control dependencies unavailable")
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(mock_drive_transport_test test/mock_drive_transport_test.cpp)
  target_compile_features(mock_drive_transport_test PRIVATE cxx_std_17)
  target_link_libraries(mock_drive_transport_test PRIVATE lb_drive_transport)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mock_drive_transport_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME mock_drive_transport_test COMMAND mock_drive_transport_test)

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(phase2_contract test/test_phase2_contract.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(lb_drive_transport)
ament_package()
