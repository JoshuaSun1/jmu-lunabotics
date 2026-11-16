cmake_minimum_required(VERSION 3.16)
project(lb_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/arena_scene_markers.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_pytest_test(navigation_mock_config test/test_navigation_mock_config.py)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(nav2_batch1_contract test/test_nav2_batch1_contract.py)
  ament_add_pytest_test(nav2_n3_contract test/test_nav2_n3_contract.py)
  ament_add_pytest_test(nav2_n4_costmaps test/test_nav2_n4_costmaps.py)
  ament_add_pytest_test(nav2_n4_lifecycle_runtime test/test_nav2_n4_lifecycle_runtime.py)
  ament_add_pytest_test(arena_map_contract test/test_arena_map_contract.py)
  ament_add_pytest_test(map_server_contract test/test_map_server_contract.py)
  ament_add_pytest_test(arena_scene_markers_test test/test_arena_scene_markers.py)
endif()

ament_package()
