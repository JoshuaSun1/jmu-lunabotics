cmake_minimum_required(VERSION 3.16)
project(lb_safety)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/navigation_command_gate.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_command_gate_test test/test_navigation_command_gate.py)
  ament_add_pytest_test(navigation_command_gate_runtime_test test/test_navigation_command_gate_runtime.py)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
