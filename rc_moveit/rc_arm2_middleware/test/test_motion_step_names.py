import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ACTION_SETS_PATH = PACKAGE_ROOT / "config" / "action_sets.yaml"
MIDDLEWARE_PATH = PACKAGE_ROOT / "rc_arm2_middleware" / "arm2_middleware_node.py"


def _configured_step_types() -> set[str]:
    config = yaml.safe_load(ACTION_SETS_PATH.read_text(encoding="utf-8"))
    step_types: set[str] = set()
    for action_set in config["action_sets"]:
        for step in action_set["steps"]:
            step_types.add(step["type"])
    return step_types


def test_action_sets_use_suffixless_j5_motion_step_types() -> None:
    step_types = _configured_step_types()
    motion_types = {step_type for step_type in step_types if step_type.startswith("move_")}

    assert all("_mrl" not in step_type for step_type in motion_types)
    assert all("_mf" not in step_type for step_type in motion_types)
    assert {
        "move_fixed_pose",
        "move_target_offset",
        "move_target_offset_cartesian",
        "move_fixed_path",
        "move_target_offset_path",
    }.issubset(motion_types)


def test_middleware_supports_only_suffixless_j5_motion_step_types() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")

    assert "move_fixed_pose_mrl" not in source
    assert "move_target_offset_mrl" not in source
    assert "move_target_offset_path_mrl" not in source
    assert "move_fixed_pose_mf" not in source
    assert "move_target_offset_mf" not in source
    assert "move_target_offset_path_mf" not in source
    assert "move_fixed_path_mf" not in source
    assert '"move_target_offset_path"' in source
    assert '"move_fixed_path"' in source


def test_target_offset_j5_target_includes_target_point_y() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")

    offset_branch_start = source.index('if step.step_type in {\n            "move_target_offset",')
    offset_branch_end = source.index('"move_fixed_pose",', offset_branch_start)
    offset_branch = source[offset_branch_start:offset_branch_end]

    assert '"move_target_offset_cartesian",' in offset_branch
    assert (
        "j5_target_pos = float(target_point.y) + float(step.j5_target_pos)"
        in offset_branch
    )


def test_target_offset_path_j5_target_includes_target_point_y() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")

    path_branch_start = source.index('if step.step_type == "move_target_offset_path"')
    path_branch_end = source.index('if step.step_type == "move_fixed_path"', path_branch_start)
    path_branch = source[path_branch_start:path_branch_end]

    assert (
        "j5_target_pos = float(target_point.y) + float(step.j5_target_pos)"
        in path_branch
    )


def test_fixed_pose_j5_target_does_not_include_target_point_y() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")

    fixed_pose_branch_start = source.index('if step.step_type in {\n            "move_fixed_pose",')
    fixed_pose_branch_end = source.index('if step.step_type == "move_target_offset_path"', fixed_pose_branch_start)
    fixed_pose_branch = source[fixed_pose_branch_start:fixed_pose_branch_end]

    assert "j5_target_pos = step.j5_target_pos" in fixed_pose_branch
    assert "target_point.y" not in fixed_pose_branch


def test_update_target_point_only_allows_fallback_on_timeout() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")

    sample_call = source[
        source.index("if len(run.target_point_samples)"):
        source.index("def _on_run_action_set")
    ]
    timeout_call = source[
        source.index(
            "if self._state == MiddlewareState.WAITING_TARGET_POINT_UPDATE",
            source.index("def _on_timer"),
        ):
        source.index("if timeout_skip_current_step")
    ]

    assert "allow_fallback=False" in sample_call
    assert "allow_fallback=True" in timeout_call


def test_update_target_point_parser_reads_finite_fallback_xyz() -> None:
    source = MIDDLEWARE_PATH.read_text(encoding="utf-8")
    parser_branch = source[
        source.index('if step_type == "update_target_point"'):
        source.index('if step_type == "send_can_frame"')
    ]

    assert 'raw_step.get("fallback_xyz")' in parser_branch
    assert '"fallback_xyz"' in parser_branch
    assert "math.isfinite" in parser_branch


def test_all_update_target_point_steps_have_finite_xyz_fallbacks() -> None:
    config = yaml.safe_load(ACTION_SETS_PATH.read_text(encoding="utf-8"))
    update_steps = [
        step
        for action_set in config["action_sets"]
        for step in action_set["steps"]
        if step["type"] == "update_target_point"
    ]

    assert update_steps
    for step in update_steps:
        fallback_xyz = step.get("fallback_xyz")
        assert isinstance(fallback_xyz, list)
        assert len(fallback_xyz) == 3
        assert all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in fallback_xyz
        )
