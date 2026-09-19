cmake_minimum_required(VERSION 3.8)
project(rc_arm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    trajectory_execution_report_test
    tests/test_trajectory_execution_report.py
    TIMEOUT 120
  )
endif()

ament_package()
