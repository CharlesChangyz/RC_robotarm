from pathlib import Path


REPO = Path(__file__).resolve().parents[3]


def read_rel(path: str) -> str:
    return (REPO / path).read_text(encoding="utf-8")


def test_robot_launch_wires_cartesian_motion_topic_and_defaults():
    launch = read_rel("rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py")

    assert "middleware_cartesian_motion_target_topic" in launch
    assert "middleware_cartesian_motion_path_topic" in launch
    assert "target_pose_executor_cartesian_max_step" in launch
    assert "target_pose_executor_cartesian_min_fraction" in launch
    assert "--middleware-cartesian-target-topic" in launch
    assert "--middleware-cartesian-path-topic" in launch
    assert "--cartesian-max-step" in launch
    assert "--cartesian-min-fraction" in launch


def test_executor_supports_cartesian_requests():
    executor = read_rel("rc_moveit/rc_arm_moveit_config/launch/target_pose_moveit_executor.py")

    assert "Arm2TargetPath" in executor
    assert "GetCartesianPath" in executor
    assert "ExecuteTrajectory" in executor
    assert "use_cartesian" in executor
    assert "_compute_cartesian_path" in executor
    assert "_on_middleware_cartesian_path" in executor
    assert "waypoint_count" in executor


def test_middleware_has_explicit_cartesian_action_type():
    middleware = read_rel("rc_moveit/rc_arm2_middleware/rc_arm2_middleware/arm2_middleware_node.py")

    assert "cartesian_motion_target_topic" in middleware
    assert '"move_target_offset_cartesian"' in middleware
    assert '"move_fixed_pose_cartesian"' in middleware
    assert "cartesian=step.step_type.endswith(\"_cartesian\")" in middleware
    assert "self._cartesian_motion_target_pub if cartesian" in middleware


def test_arm_msgs_declares_target_path_message():
    cmake = read_rel("rc_moveit/arm_msgs/CMakeLists.txt")
    msg = read_rel("rc_moveit/arm_msgs/msg/Arm2TargetPath.msg")

    assert '"msg/Arm2TargetPath.msg"' in cmake
    assert msg.splitlines() == [
        "Arm2TargetPoint[] waypoints",
        "float64 blend_radius",
    ]
