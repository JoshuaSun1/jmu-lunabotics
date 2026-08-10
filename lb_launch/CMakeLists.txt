cmake_minimum_required(VERSION 3.16)
project(lb_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_scaffold test/test_scaffold.py)
  ament_add_pytest_test(phase1_launch_contract test/test_phase1_launch_contract.py)
endif()

ament_package()
