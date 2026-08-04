cmake_minimum_required(VERSION 3.16)
project(lunabot_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_scaffold test/test_scaffold.py)
endif()

ament_package()
