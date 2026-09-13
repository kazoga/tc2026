cmake_minimum_required(VERSION 3.8)
project(obstacle_route_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params
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
  DIRECTORY tools/terrain3d
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "node_modules" EXCLUDE
)

install(
  FILES
    README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_localization_pose.py
    scripts/generate_pylon_world.py
    scripts/generate_waypoints.py
    scripts/odom_tf_broadcaster.py
    scripts/road_generator.py
    tools/generate_route_assets.py
    tools/audit_full_course.py
    tools/prepare_full_course.py
    tools/reconstruct_full_course.py
    tools/preview_full_course.py
    tools/render_full_course_views.py
    tools/clear_route_corridor.py
    tools/prepare_waypoint_evaluation.py
    tools/plot_waypoint_evaluation.py
    tools/prepare_fastlio_trial.py
    tools/lio_sensor_adapter_node.py
    tools/plot_lio_evaluation.py
    tools/plot_urg_evaluation.py
    tools/lio_map_recorder_node.py
    tools/evaluate_lio_map.py
    tools/review_lio_noise_trials.py
    tools/review_full_lio_trial.py
    tools/review_fusion_trial.py
    tools/evaluate_fusion_replay.py
    tools/evaluate_heading_replay.py
    tools/build_geographic_trial.py
    tools/gnss_simulator_node.py
    tools/build_terrain_trial.py
    tools/evaluate_terrain_trial.py
    tools/preview_terrain_trial.py
    tools/prepare_gazebo_runtime.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    tools/gnss_environment_core.py
    tools/lio_noise_core.py
    tools/lio_evaluation_core.py
    tools/geographic_reconstruction_core.py
    scripts/road_geometry.py
    tools/activate_gazebo_runtime.bash
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_route_corridor tests/test_route_corridor.py)
  ament_add_pytest_test(test_full_lio_review tests/test_full_lio_review.py)
  ament_add_pytest_test(test_lio_noise tests/test_lio_noise.py)
  ament_add_pytest_test(test_lio_map tests/test_lio_map.py)
  ament_add_pytest_test(test_lio_simulator tests/test_lio_simulator.py)
  ament_add_pytest_test(test_gnss_environment tests/test_gnss_environment.py)
  ament_add_pytest_test(test_geographic_reconstruction tests/test_geographic_reconstruction.py)
  ament_add_pytest_test(test_road_geometry tests/test_road_geometry.py)
  ament_add_pytest_test(test_generators tests/test_generators.py)
  ament_add_pytest_test(test_gnss_simulator tests/test_gnss_simulator.py)
  ament_add_pytest_test(test_terrain_trial tests/test_terrain_trial.py)
endif()

ament_package()
