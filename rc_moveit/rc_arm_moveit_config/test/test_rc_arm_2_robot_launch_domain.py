from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
LAUNCH_FILE = (
    ROOT
    / "rc_moveit"
    / "rc_arm_moveit_config"
    / "launch"
    / "rc_arm_2_robot.launch.py"
)


class RcArm2RobotLaunchDomainTest(unittest.TestCase):
    def test_robot_launch_sets_ros_domain_for_all_child_processes(self):
        launch_text = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertIn("SetEnvironmentVariable", launch_text)
        self.assertIn("EnvironmentVariable", launch_text)
        self.assertIn("'ros_domain_id'", launch_text)
        self.assertIn("'ROS_DOMAIN_ID'", launch_text)


if __name__ == "__main__":
    unittest.main()
