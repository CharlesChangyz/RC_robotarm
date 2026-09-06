#!/usr/bin/env python3
"""Static checks for target_pose_moveit_executor planner defaults."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
EXECUTOR = PACKAGE_DIR / "launch" / "target_pose_moveit_executor.py"
ROBOT_LAUNCH = PACKAGE_DIR / "launch" / "rc_arm_2_robot.launch.py"


class TargetPoseSinglePlannerDefaultsTest(unittest.TestCase):
    def test_executor_cli_defaults_single_point_to_pilz_ptp(self) -> None:
        tree = ast.parse(EXECUTOR.read_text(encoding="utf-8"))
        add_argument_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ]

        defaults = {}
        for call in add_argument_calls:
            option = call.args[0].value
            for keyword in call.keywords:
                if keyword.arg == "default" and isinstance(keyword.value, ast.Constant):
                    defaults[option] = keyword.value.value

        self.assertEqual(
            defaults.get("--single-pipeline-id"),
            "pilz_industrial_motion_planner",
        )
        self.assertEqual(defaults.get("--single-planner-id"), "PTP")

    def test_launch_passes_single_point_planner_arguments(self) -> None:
        text = ROBOT_LAUNCH.read_text(encoding="utf-8")
        self.assertIn("target_pose_executor_single_pipeline_id", text)
        self.assertIn("target_pose_executor_single_planner_id", text)
        self.assertIn("'--single-pipeline-id'", text)
        self.assertIn("'--single-planner-id'", text)


if __name__ == "__main__":
    unittest.main()
