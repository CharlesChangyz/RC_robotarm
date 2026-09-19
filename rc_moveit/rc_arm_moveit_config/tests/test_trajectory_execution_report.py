from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"
sys.path.insert(0, str(LAUNCH_DIR))

from trajectory_execution_report import (  # noqa: E402
    TrajectoryReportWriter,
    classify_motion,
    classify_status,
    compute_metrics,
    reorder_joint_values,
    should_generate_report,
)


def _samples():
    return [
        {
            "stamp_sec": 100.0,
            "elapsed_sec": 0.0,
            "desired_position": [1.0, 2.0],
            "actual_position": [0.9, 2.2],
            "position_error": [0.1, -0.2],
            "desired_velocity": [0.5, 0.25],
            "actual_velocity": [0.4, 0.35],
            "velocity_error": [0.1, -0.1],
            "desired_eef_position": [0.1, 0.2, 0.3],
            "actual_eef_position": [0.09, 0.2, 0.3],
            "eef_position_error_m": 0.01,
            "eef_orientation_error_rad": 0.02,
            "eef_pitch_error_rad": 0.015,
        },
        {
            "stamp_sec": 101.5,
            "elapsed_sec": 1.5,
            "desired_position": [1.4, 2.4],
            "actual_position": [1.1, 2.8],
            "position_error": [0.3, -0.4],
            "desired_velocity": [0.0, 0.0],
            "actual_velocity": [-0.2, 0.1],
            "velocity_error": [0.2, -0.1],
            "desired_eef_position": [0.2, 0.3, 0.4],
            "actual_eef_position": [0.18, 0.3, 0.4],
            "eef_position_error_m": 0.02,
            "eef_orientation_error_rad": 0.04,
            "eef_pitch_error_rad": 0.03,
        },
    ]


def _report(run_id: str):
    return {
        "metadata": {
            "run_id": run_id,
            "started_at": "2026-09-19T12:00:00.000+08:00",
            "finished_at": "2026-09-19T12:00:02.000+08:00",
            "label": "test",
            "source": "middleware",
            "motion_type": "single",
            "pipeline_id": "pilz_industrial_motion_planner",
            "planner_id": "PTP",
            "execution_id": 7,
            "status": "succeeded",
            "error_code": 1,
            "planning_time_sec": 0.12,
            "target_pose": {},
            "waypoints": [],
        },
        "joint_names": ["j1", "j2"],
        "samples": _samples(),
        "planned_duration_sec": 1.25,
        "planned_trajectory": [
            {
                "segment": 0,
                "point": 0,
                "time_from_start_sec": 0.0,
                "joint_names": ["j2", "j1"],
                "positions": [2.0, 1.0],
                "velocities": [],
                "accelerations": [],
            }
        ],
    }


def test_compute_metrics_known_values():
    metrics = compute_metrics(["j1", "j2"], _samples(), planned_duration_sec=1.25)
    assert metrics["sample_count"] == 2
    assert metrics["execution_duration_sec"] == 1.5
    assert metrics["duration_ratio"] == 1.2
    assert math.isclose(metrics["joint_position_error_rad"]["rmse"], math.sqrt(0.075))
    assert metrics["joint_position_error_rad"]["max_abs"] == 0.4
    assert metrics["per_joint"]["j1"]["position_error_rad"]["final"] == 0.3
    assert metrics["eef_position_error_m"]["final"] == 0.02


def test_empty_and_single_sample_metrics_are_stable():
    empty = compute_metrics(["j1"], [], planned_duration_sec=0.0)
    assert empty["sample_count"] == 0
    assert empty["joint_position_error_rad"]["rmse"] is None
    assert empty["duration_ratio"] is None

    sample = dict(_samples()[0])
    sample["desired_velocity"] = []
    sample["actual_velocity"] = []
    sample["velocity_error"] = []
    single = compute_metrics(["j1", "j2"], [sample], planned_duration_sec=1.0)
    assert single["execution_duration_sec"] == 0.0
    assert single["joint_velocity_error_rad_s"]["rmse"] is None


def test_reorder_feedback_and_report_gate():
    assert reorder_joint_values(["j1", "j2"], ["j2", "j1"], [2.0, 1.0]) == [1.0, 2.0]
    fallback = reorder_joint_values(["j1", "j2"], ["j1", "j2"], [], [0.1, 0.2])
    assert fallback == [0.1, 0.2]
    assert not should_generate_report(True, [])
    assert not should_generate_report(False, _samples())
    assert should_generate_report(True, _samples())


def test_motion_and_result_classification():
    assert classify_motion(False, False) == "single"
    assert classify_motion(False, True) == "cartesian"
    assert classify_motion(True, False) == "sequence"
    assert classify_status(True, 1, "goal_done", -7) == "succeeded"
    assert classify_status(False, -4, "goal_done", -7) == "failed"
    assert classify_status(False, -7, "goal_done", -7) == "preempted"
    assert classify_status(False, -4, "sequence_goal_preempted", -7) == "preempted"


def test_writer_generates_complete_offline_report(tmp_path):
    writer = TrajectoryReportWriter(str(tmp_path))
    path = writer.submit(_report("run-a")).result(timeout=60)
    writer.close()

    assert path.is_file()
    run_dir = path.parent
    assert (run_dir / "samples.csv").is_file()
    assert (run_dir / "planned_trajectory.csv").is_file()
    assert (run_dir / "metadata.json").is_file()
    assert (run_dir / "joint_position_tracking.png").stat().st_size > 0
    assert (run_dir / "eef_path_3d.png").stat().st_size > 0
    assert (tmp_path / "index.html").is_file()

    metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["metrics"]["sample_count"] == 2
    with (run_dir / "planned_trajectory.csv").open(encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert row["j1_position_rad"] == "1.0"
    assert row["j2_position_rad"] == "2.0"


def test_writer_serializes_concurrent_submissions_without_losing_summary_rows(tmp_path):
    writer = TrajectoryReportWriter(str(tmp_path))
    futures = [writer.submit(_report(f"run-{index}")) for index in range(3)]
    for future in futures:
        future.result(timeout=60)
    writer.close()

    with (tmp_path / "summary.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert {row["run_id"] for row in rows} == {"run-0", "run-1", "run-2"}
