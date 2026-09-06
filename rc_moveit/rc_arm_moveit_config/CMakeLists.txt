cmake_minimum_required(VERSION 3.8)
project(rc_arm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_cartesian_motion_defaults
    test/test_cartesian_motion_defaults.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_rc_arm_2_robot_launch_domain
    test/test_rc_arm_2_robot_launch_domain.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_rc_arm_world_pitch_kinematics
    test/test_rc_arm_world_pitch_kinematics.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_target_pose_single_planner_defaults
    test/test_target_pose_single_planner_defaults.py
    TIMEOUT 60
  )
endif()

ament_package()
