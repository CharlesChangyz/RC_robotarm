cmake_minimum_required(VERSION 3.8)
project(rc_arm_description)

find_package(ament_cmake REQUIRED)

# Auto-generate Pinocchio URDF from the authoritative xacro source.
find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(RC_ARM2_XACRO
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/rc_arm_2/rc_arm_2.urdf.xacro)
set(RC_ARM2_ROS2_CONTROL_XACRO
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/rc_arm_2/rc_arm_2_ros2_control.xacro)
set(RC_ARM2_REAL_HARDWARE_CONFIG
  ${CMAKE_CURRENT_SOURCE_DIR}/config/rc_arm_2/rc_arm_2_hardware.real.yaml)
set(RC_ARM2_MUJOCO_HARDWARE_CONFIG
  ${CMAKE_CURRENT_SOURCE_DIR}/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml)
set(GENERATED_PINOCCHIO_URDF
  ${CMAKE_CURRENT_BINARY_DIR}/generated/urdf/rc_arm_2/rc_arm_2.pinocchio.urdf)

add_custom_command(
  OUTPUT ${GENERATED_PINOCCHIO_URDF}
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${CMAKE_CURRENT_BINARY_DIR}/generated/urdf/rc_arm_2
  COMMAND ${XACRO_EXECUTABLE}
          ${RC_ARM2_XACRO}
          hardware_config_file:=${RC_ARM2_REAL_HARDWARE_CONFIG}
          -o ${GENERATED_PINOCCHIO_URDF}
  DEPENDS
    ${RC_ARM2_XACRO}
    ${RC_ARM2_ROS2_CONTROL_XACRO}
    ${RC_ARM2_REAL_HARDWARE_CONFIG}
    ${RC_ARM2_MUJOCO_HARDWARE_CONFIG}
  COMMENT "Generating rc_arm_2.pinocchio.urdf from rc_arm_2.urdf.xacro"
  VERBATIM
)

add_custom_target(generate_pinocchio_urdf ALL
  DEPENDS ${GENERATED_PINOCCHIO_URDF})

# Install source directories, but exclude the generated URDF from source tree install.
install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "rc_arm_2.pinocchio.urdf" EXCLUDE
)

# Install the generated Pinocchio URDF to the canonical package path.
install(
  FILES ${GENERATED_PINOCCHIO_URDF}
  DESTINATION share/${PROJECT_NAME}/urdf/rc_arm_2
  RENAME rc_arm_2.pinocchio.urdf
)

ament_package()
