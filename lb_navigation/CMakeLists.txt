cmake_minimum_required(VERSION 3.16)
project(lb_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(arena_map_contract test/test_arena_map_contract.py)
  ament_add_pytest_test(map_server_contract test/test_map_server_contract.py)
endif()

ament_package()
