cmake_minimum_required(VERSION 3.8)
project(obstacle_route_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_amcl_pose.py
    scripts/generate_pylon_world.py
    scripts/generate_waypoints.py
    scripts/odom_tf_broadcaster.py
    scripts/road_generator.py
    tools/generate_route_assets.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/road_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_pytest_test(test_road_geometry tests/test_road_geometry.py)
  ament_add_pytest_test(test_generators tests/test_generators.py)
endif()

ament_package()
