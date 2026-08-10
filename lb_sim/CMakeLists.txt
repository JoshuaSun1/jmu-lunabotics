cmake_minimum_required(VERSION 3.16)
project(lb_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake QUIET)
  find_package(controller_manager_msgs QUIET)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(mock_launch_contract test/test_mock_launch_contract.py)

  find_program(XACRO_EXECUTABLE xacro)
  if(launch_testing_ament_cmake_FOUND AND controller_manager_msgs_FOUND AND XACRO_EXECUTABLE)
    add_launch_test(test/test_mock_robot_launch.py TIMEOUT 90)
  else()
    message(STATUS "Skipping lb_sim runtime launch test: missing xacro or ROS 2 control test dependencies")
  endif()
endif()

ament_package()
