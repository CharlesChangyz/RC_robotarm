#!/usr/bin/env python3
"""Offline trajectory execution reports for the rc_arm_2 MoveIt executor."""

from __future__ import annotations

import csv
from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime
import html
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Dict, Iterable, List, Mapping, Optional, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SUMMARY_FIELDS = [
    "run_id",
    "started_at",
    "label",
    "source",
    "motion_type",
    "pipeline_id",
    "planner_id",
    "execution_id",
    "status",
    "error_code",
    "planning_time_sec",
    "planned_duration_sec",
    "execution_duration_sec",
    "duration_ratio",
    "joint_position_rmse_rad",
    "joint_position_max_abs_rad",
    "eef_position_rmse_m",
    "eef_position_max_m",
    "eef_position_final_m",
    "eef_orientation_final_rad",
    "eef_pitch_final_rad",
    "sample_count",
    "report_path",
]


def should_generate_report(enabled: bool, samples: Sequence[Mapping]) -> bool:
    """A planning request becomes an execution report only after controller feedback."""
    return bool(enabled and samples)


def classify_motion(use_sequence: bool, use_cartesian: bool) -> str:
    if use_sequence:
        return "sequence"
    if use_cartesian:
        return "cartesian"
    return "single"


def classify_status(success: bool, error_code: int, reason: str, preempt_code: int) -> str:
    if int(error_code) == int(preempt_code) or "preempt" in str(reason).lower():
        return "preempted"
    return "succeeded" if success else "failed"


def reorder_joint_values(
    target_names: Sequence[str],
    incoming_names: Sequence[str],
    values: Sequence[float],
    fallback: Optional[Sequence[float]] = None,
) -> List[float]:
    index_by_name = {name: index for index, name in enumerate(incoming_names)}
    missing = [name for name in target_names if name not in index_by_name]
    if missing:
        raise ValueError("feedback missing joints: %s" % ", ".join(missing))
    result = []
    for target_index, name in enumerate(target_names):
        source_index = index_by_name[name]
        if source_index < len(values):
            result.append(float(values[source_index]))
        elif fallback is not None and target_index < len(fallback):
            result.append(float(fallback[target_index]))
        else:
            result.append(math.nan)
    return result


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_figure(path: Path, figure) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    os.close(fd)
    try:
        figure.savefig(temp_name, format="png", dpi=140, bbox_inches="tight")
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise
    finally:
        plt.close(figure)


def _array(samples: Sequence[Mapping], key: str, width: int) -> np.ndarray:
    rows = []
    for sample in samples:
        values = list(sample.get(key, []))
        row = []
        for index in range(width):
            try:
                value = float(values[index])
            except (IndexError, TypeError, ValueError):
                value = math.nan
            row.append(value)
        rows.append(row)
    return np.asarray(rows, dtype=float)


def _scalar_array(samples: Sequence[Mapping], key: str) -> np.ndarray:
    values = []
    for sample in samples:
        try:
            values.append(float(sample.get(key, math.nan)))
        except (TypeError, ValueError):
            values.append(math.nan)
    return np.asarray(values, dtype=float)


def _stats(values: np.ndarray) -> Dict[str, Optional[float]]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {"rmse": None, "mae": None, "max_abs": None, "final": None}
    return {
        "rmse": float(np.sqrt(np.mean(np.square(finite)))),
        "mae": float(np.mean(np.abs(finite))),
        "max_abs": float(np.max(np.abs(finite))),
        "final": float(finite[-1]),
    }


def compute_metrics(
    joint_names: Sequence[str],
    samples: Sequence[Mapping],
    planned_duration_sec: float = 0.0,
) -> Dict:
    """Compute deterministic report metrics from normalized controller samples."""
    count = len(samples)
    width = len(joint_names)
    position_error = _array(samples, "position_error", width)
    velocity_error = _array(samples, "velocity_error", width)

    per_joint = {}
    for index, joint_name in enumerate(joint_names):
        per_joint[joint_name] = {
            "position_error_rad": _stats(position_error[:, index] if count else np.array([])),
            "velocity_error_rad_s": _stats(velocity_error[:, index] if count else np.array([])),
        }

    elapsed = _scalar_array(samples, "elapsed_sec")
    finite_elapsed = elapsed[np.isfinite(elapsed)]
    execution_duration = (
        max(0.0, float(finite_elapsed[-1] - finite_elapsed[0]))
        if finite_elapsed.size >= 2
        else 0.0
    )
    duration_ratio = (
        execution_duration / float(planned_duration_sec)
        if float(planned_duration_sec) > 0.0
        else None
    )

    eef_position = _stats(_scalar_array(samples, "eef_position_error_m"))
    eef_orientation = _stats(_scalar_array(samples, "eef_orientation_error_rad"))
    eef_pitch = _stats(_scalar_array(samples, "eef_pitch_error_rad"))
    return {
        "sample_count": count,
        "execution_duration_sec": execution_duration,
        "planned_duration_sec": max(0.0, float(planned_duration_sec)),
        "duration_ratio": duration_ratio,
        "joint_position_error_rad": _stats(position_error.reshape(-1)),
        "joint_velocity_error_rad_s": _stats(velocity_error.reshape(-1)),
        "per_joint": per_joint,
        "eef_position_error_m": eef_position,
        "eef_orientation_error_rad": eef_orientation,
        "eef_pitch_error_rad": eef_pitch,
    }


def _fmt(value, digits: int = 6) -> str:
    if value is None:
        return "NA"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return html.escape(str(value))
    if not math.isfinite(number):
        return "NA"
    return f"{number:.{digits}f}"


def _sample_rows(joint_names: Sequence[str], samples: Sequence[Mapping]) -> tuple[List[str], List[Dict]]:
    fields = ["stamp_sec", "elapsed_sec"]
    for joint in joint_names:
        fields.extend(
            [
                f"{joint}_desired_position_rad",
                f"{joint}_actual_position_rad",
                f"{joint}_position_error_rad",
                f"{joint}_desired_velocity_rad_s",
                f"{joint}_actual_velocity_rad_s",
                f"{joint}_velocity_error_rad_s",
            ]
        )
    fields.extend(
        [
            "desired_eef_x_m", "desired_eef_y_m", "desired_eef_z_m",
            "actual_eef_x_m", "actual_eef_y_m", "actual_eef_z_m",
            "desired_eef_pitch_rad", "actual_eef_pitch_rad",
            "eef_position_error_m", "eef_orientation_error_rad", "eef_pitch_error_rad",
        ]
    )
    rows = []
    for sample in samples:
        row = {"stamp_sec": sample.get("stamp_sec"), "elapsed_sec": sample.get("elapsed_sec")}
        for index, joint in enumerate(joint_names):
            for source, suffix in (
                ("desired_position", "desired_position_rad"),
                ("actual_position", "actual_position_rad"),
                ("position_error", "position_error_rad"),
                ("desired_velocity", "desired_velocity_rad_s"),
                ("actual_velocity", "actual_velocity_rad_s"),
                ("velocity_error", "velocity_error_rad_s"),
            ):
                values = sample.get(source, [])
                row[f"{joint}_{suffix}"] = values[index] if index < len(values) else ""
        for prefix, key in (("desired", "desired_eef_position"), ("actual", "actual_eef_position")):
            values = sample.get(key, [])
            for index, axis in enumerate("xyz"):
                row[f"{prefix}_eef_{axis}_m"] = values[index] if index < len(values) else ""
        row["eef_position_error_m"] = sample.get("eef_position_error_m")
        row["desired_eef_pitch_rad"] = sample.get("desired_eef_pitch_rad")
        row["actual_eef_pitch_rad"] = sample.get("actual_eef_pitch_rad")
        row["eef_orientation_error_rad"] = sample.get("eef_orientation_error_rad")
        row["eef_pitch_error_rad"] = sample.get("eef_pitch_error_rad")
        rows.append(row)
    return fields, rows


def _planned_rows(joint_names: Sequence[str], planned: Sequence[Mapping]) -> tuple[List[str], List[Dict]]:
    fields = ["segment", "point", "time_from_start_sec"]
    for joint in joint_names:
        fields.extend(
            [
                f"{joint}_position_rad",
                f"{joint}_velocity_rad_s",
                f"{joint}_acceleration_rad_s2",
            ]
        )
    rows = []
    for item in planned:
        row = {
            "segment": item.get("segment", 0),
            "point": item.get("point", 0),
            "time_from_start_sec": item.get("time_from_start_sec", 0.0),
        }
        item_names = list(item.get("joint_names", joint_names))
        name_index = {name: index for index, name in enumerate(item_names)}
        for joint in joint_names:
            source_index = name_index.get(joint)
            for source, suffix in (
                ("positions", "position_rad"),
                ("velocities", "velocity_rad_s"),
                ("accelerations", "acceleration_rad_s2"),
            ):
                values = item.get(source, [])
                row[f"{joint}_{suffix}"] = (
                    values[source_index]
                    if source_index is not None and source_index < len(values)
                    else ""
                )
        rows.append(row)
    return fields, rows


def _line_plots(run_dir: Path, joint_names: Sequence[str], samples: Sequence[Mapping]) -> List[str]:
    if not samples:
        return []
    times = _scalar_array(samples, "elapsed_sec")
    desired_position = _array(samples, "desired_position", len(joint_names))
    actual_position = _array(samples, "actual_position", len(joint_names))
    position_error = _array(samples, "position_error", len(joint_names))
    desired_velocity = _array(samples, "desired_velocity", len(joint_names))
    actual_velocity = _array(samples, "actual_velocity", len(joint_names))
    desired_eef = _array(samples, "desired_eef_position", 3)
    actual_eef = _array(samples, "actual_eef_position", 3)

    images = []
    figure, axes = plt.subplots(len(joint_names), 1, figsize=(10, max(3, 2.5 * len(joint_names))), sharex=True)
    axes = np.atleast_1d(axes)
    for index, (axis, joint) in enumerate(zip(axes, joint_names)):
        axis.plot(times, desired_position[:, index], label="desired", linewidth=1.4)
        axis.plot(times, actual_position[:, index], label="actual", linewidth=1.1)
        axis.set_ylabel(f"{joint} (rad)")
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Execution time (s)")
    figure.suptitle("Joint position tracking")
    name = "joint_position_tracking.png"
    _atomic_figure(run_dir / name, figure)
    images.append(name)

    figure, axes = plt.subplots(len(joint_names), 1, figsize=(10, max(3, 2.5 * len(joint_names))), sharex=True)
    axes = np.atleast_1d(axes)
    for index, (axis, joint) in enumerate(zip(axes, joint_names)):
        axis.plot(times, position_error[:, index], linewidth=1.2)
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_ylabel(f"{joint} (rad)")
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel("Execution time (s)")
    figure.suptitle("Joint position error (desired - actual)")
    name = "joint_position_error.png"
    _atomic_figure(run_dir / name, figure)
    images.append(name)

    figure, axes = plt.subplots(len(joint_names), 1, figsize=(10, max(3, 2.5 * len(joint_names))), sharex=True)
    axes = np.atleast_1d(axes)
    for index, (axis, joint) in enumerate(zip(axes, joint_names)):
        axis.plot(times, desired_velocity[:, index], label="desired", linewidth=1.4)
        axis.plot(times, actual_velocity[:, index], label="actual", linewidth=1.1)
        axis.set_ylabel(f"{joint} (rad/s)")
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Execution time (s)")
    figure.suptitle("Joint velocity tracking")
    name = "joint_velocity_tracking.png"
    _atomic_figure(run_dir / name, figure)
    images.append(name)

    figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for index, axis_name in enumerate("XYZ"):
        axis = axes.flat[index]
        axis.plot(times, desired_eef[:, index], label="desired", linewidth=1.4)
        axis.plot(times, actual_eef[:, index], label="actual", linewidth=1.1)
        axis.set_ylabel(f"{axis_name} (m)")
        axis.grid(True, alpha=0.25)
    error_axis = axes.flat[3]
    error_axis.plot(times, _scalar_array(samples, "eef_position_error_m") * 1000.0)
    error_axis.set_ylabel("Position error (mm)")
    error_axis.grid(True, alpha=0.25)
    for axis in axes[-1, :]:
        axis.set_xlabel("Execution time (s)")
    axes.flat[0].legend(loc="best")
    figure.suptitle("End-effector tracking")
    name = "eef_tracking.png"
    _atomic_figure(run_dir / name, figure)
    images.append(name)

    figure = plt.figure(figsize=(8, 7))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(desired_eef[:, 0], desired_eef[:, 1], desired_eef[:, 2], label="desired")
    axis.plot(actual_eef[:, 0], actual_eef[:, 1], actual_eef[:, 2], label="actual")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    axis.legend(loc="best")
    axis.set_title("End-effector 3D path")
    name = "eef_path_3d.png"
    _atomic_figure(run_dir / name, figure)
    images.append(name)
    return images


def _joint_table(metrics: Mapping) -> str:
    rows = []
    for joint, item in metrics.get("per_joint", {}).items():
        position = item["position_error_rad"]
        velocity = item["velocity_error_rad_s"]
        rows.append(
            "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                html.escape(joint),
                _fmt(position["rmse"]),
                _fmt(position["mae"]),
                _fmt(position["max_abs"]),
                _fmt(position["final"]),
                _fmt(velocity["rmse"]),
                _fmt(velocity["max_abs"]),
            )
        )
    return "".join(rows)


def _report_html(metadata: Mapping, metrics: Mapping, images: Sequence[str]) -> str:
    eef_position = metrics["eef_position_error_m"]
    eef_orientation = metrics["eef_orientation_error_rad"]
    eef_pitch = metrics["eef_pitch_error_rad"]
    target_final = metrics.get("target_final_error", {})
    image_html = "".join(
        f'<section><img src="{html.escape(name)}" alt="{html.escape(name)}"></section>'
        for name in images
    )
    metadata_rows = "".join(
        "<tr><th>%s</th><td>%s</td></tr>" % (html.escape(str(key)), html.escape(str(value)))
        for key, value in metadata.items()
        if key not in {"target_pose", "waypoints"}
    )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>Trajectory report {html.escape(str(metadata.get('run_id', '')))}</title>
<style>body{{font-family:sans-serif;max-width:1200px;margin:2rem auto;padding:0 1rem;color:#222}}table{{border-collapse:collapse;width:100%;margin:1rem 0}}th,td{{border:1px solid #ccc;padding:.45rem;text-align:right}}th:first-child,td:first-child{{text-align:left}}img{{max-width:100%;height:auto}}.ok{{color:#087f23}}.failed{{color:#b00020}}code{{background:#eee;padding:.15rem .3rem}}</style></head>
<body><p><a href="../../../index.html">← 全部执行记录</a></p>
<h1>轨迹执行报告：{html.escape(str(metadata.get('run_id', '')))}</h1>
<h2>执行信息</h2><table>{metadata_rows}</table>
<h2>总体指标</h2><table><tbody>
<tr><th>采样数</th><td>{metrics['sample_count']}</td></tr>
<tr><th>计划 / 实际时长 (s)</th><td>{_fmt(metrics['planned_duration_sec'])} / {_fmt(metrics['execution_duration_sec'])}</td></tr>
<tr><th>实际/计划时长比</th><td>{_fmt(metrics['duration_ratio'])}</td></tr>
<tr><th>末端位置 RMSE / 最大 / 最终</th><td>{_fmt(eef_position['rmse'])} / {_fmt(eef_position['max_abs'])} / {_fmt(eef_position['final'])} m<br>{_fmt(None if eef_position['rmse'] is None else eef_position['rmse']*1000)} / {_fmt(None if eef_position['max_abs'] is None else eef_position['max_abs']*1000)} / {_fmt(None if eef_position['final'] is None else eef_position['final']*1000)} mm</td></tr>
<tr><th>末端姿态最终偏差</th><td>{_fmt(eef_orientation['final'])} rad / {_fmt(None if eef_orientation['final'] is None else math.degrees(eef_orientation['final']))}°</td></tr>
<tr><th>world-pitch 最终偏差</th><td>{_fmt(eef_pitch['final'])} rad / {_fmt(None if eef_pitch['final'] is None else math.degrees(eef_pitch['final']))}°</td></tr>
<tr><th>请求目标→最终实际位置偏差</th><td>{_fmt(target_final.get('position_m'))} m / {_fmt(None if target_final.get('position_m') is None else target_final.get('position_m')*1000)} mm</td></tr>
<tr><th>请求目标→最终实际姿态偏差</th><td>{_fmt(target_final.get('orientation_rad'))} rad / {_fmt(None if target_final.get('orientation_rad') is None else math.degrees(target_final.get('orientation_rad')))}°</td></tr>
<tr><th>请求目标→最终 world-pitch 偏差</th><td>{_fmt(target_final.get('pitch_rad'))} rad / {_fmt(None if target_final.get('pitch_rad') is None else math.degrees(target_final.get('pitch_rad')))}°</td></tr>
</tbody></table>
<h2>关节指标</h2><table><thead><tr><th>关节</th><th>位置 RMSE</th><th>位置 MAE</th><th>位置最大绝对误差</th><th>位置最终误差</th><th>速度 RMSE</th><th>速度最大绝对误差</th></tr></thead><tbody>{_joint_table(metrics)}</tbody></table>
<p>原始数据：<a href="samples.csv">samples.csv</a> · <a href="planned_trajectory.csv">planned_trajectory.csv</a> · <a href="metadata.json">metadata.json</a></p>
{image_html}</body></html>"""


def _index_html(rows: Sequence[Mapping]) -> str:
    body = []
    for row in rows:
        report_path = html.escape(str(row.get("report_path", "")))
        body.append(
            "<tr><td><a href=\"%s\">%s</a></td><td>%s</td><td>%s</td><td>%s</td>"
            "<td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                report_path,
                html.escape(str(row.get("started_at", ""))),
                html.escape(str(row.get("label", ""))),
                html.escape(str(row.get("source", ""))),
                html.escape(str(row.get("motion_type", ""))),
                html.escape(str(row.get("planner_id", ""))),
                html.escape(str(row.get("status", ""))),
                _fmt(row.get("joint_position_rmse_rad")),
                _fmt(row.get("eef_position_final_m")),
                html.escape(str(row.get("sample_count", ""))),
            )
        )
    return """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>RC Arm trajectory reports</title>
<style>body{font-family:sans-serif;max-width:1400px;margin:2rem auto;padding:0 1rem}table{border-collapse:collapse;width:100%}th,td{border:1px solid #ccc;padding:.45rem;text-align:left}tr:nth-child(even){background:#f6f6f6}</style></head><body>
<h1>RC Arm 轨迹执行记录</h1><p>完整汇总数据：<a href="summary.csv">summary.csv</a></p>
<table><thead><tr><th>时间</th><th>标签</th><th>来源</th><th>类型</th><th>Planner</th><th>状态</th><th>关节 RMSE (rad)</th><th>末端最终偏差 (m)</th><th>采样数</th></tr></thead><tbody>""" + "".join(body) + "</tbody></table></body></html>"


class TrajectoryReportWriter:
    """Serialize report generation on one background worker."""

    def __init__(self, root_dir: str) -> None:
        self.root_dir = Path(root_dir).expanduser().resolve()
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="trajectory-report")
        self._closed = False
        self._lock = threading.Lock()

    def submit(self, report: Mapping) -> Future:
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory report writer is closed")
            snapshot = deepcopy(dict(report))
            return self._executor.submit(self._write_report, snapshot)

    def close(self, wait: bool = True) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._executor.shutdown(wait=wait)

    def _write_report(self, report: Dict) -> Path:
        metadata = dict(report.get("metadata", {}))
        run_id = str(metadata["run_id"])
        joint_names = list(report.get("joint_names", []))
        samples = list(report.get("samples", []))
        planned = list(report.get("planned_trajectory", []))
        planned_duration = float(report.get("planned_duration_sec", 0.0) or 0.0)
        metrics = compute_metrics(joint_names, samples, planned_duration)
        metrics["target_final_error"] = dict(metadata.get("target_final_error", {}))

        date_part = str(metadata.get("started_at", ""))[:10] or datetime.now().strftime("%Y-%m-%d")
        run_dir = self.root_dir / "runs" / date_part / run_id
        run_dir.mkdir(parents=True, exist_ok=True)

        sample_fields, sample_rows = _sample_rows(joint_names, samples)
        _atomic_csv(run_dir / "samples.csv", sample_fields, sample_rows)
        planned_fields, planned_rows = _planned_rows(joint_names, planned)
        _atomic_csv(run_dir / "planned_trajectory.csv", planned_fields, planned_rows)
        images = _line_plots(run_dir, joint_names, samples)

        metadata["metrics"] = metrics
        _atomic_text(
            run_dir / "metadata.json",
            json.dumps(_json_safe(metadata), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        )
        _atomic_text(run_dir / "report.html", _report_html(metadata, metrics, images))

        relative_report = (run_dir / "report.html").relative_to(self.root_dir).as_posix()
        summary_row = {
            "run_id": run_id,
            "started_at": metadata.get("started_at", ""),
            "label": metadata.get("label", ""),
            "source": metadata.get("source", ""),
            "motion_type": metadata.get("motion_type", ""),
            "pipeline_id": metadata.get("pipeline_id", ""),
            "planner_id": metadata.get("planner_id", ""),
            "execution_id": metadata.get("execution_id", ""),
            "status": metadata.get("status", ""),
            "error_code": metadata.get("error_code", ""),
            "planning_time_sec": metadata.get("planning_time_sec", ""),
            "planned_duration_sec": metrics["planned_duration_sec"],
            "execution_duration_sec": metrics["execution_duration_sec"],
            "duration_ratio": metrics["duration_ratio"],
            "joint_position_rmse_rad": metrics["joint_position_error_rad"]["rmse"],
            "joint_position_max_abs_rad": metrics["joint_position_error_rad"]["max_abs"],
            "eef_position_rmse_m": metrics["eef_position_error_m"]["rmse"],
            "eef_position_max_m": metrics["eef_position_error_m"]["max_abs"],
            "eef_position_final_m": metrics["target_final_error"].get(
                "position_m", metrics["eef_position_error_m"]["final"]
            ),
            "eef_orientation_final_rad": metrics["target_final_error"].get(
                "orientation_rad", metrics["eef_orientation_error_rad"]["final"]
            ),
            "eef_pitch_final_rad": metrics["target_final_error"].get(
                "pitch_rad", metrics["eef_pitch_error_rad"]["final"]
            ),
            "sample_count": metrics["sample_count"],
            "report_path": relative_report,
        }
        self._update_index(summary_row)
        return run_dir / "report.html"

    def _update_index(self, new_row: Mapping) -> None:
        summary_path = self.root_dir / "summary.csv"
        rows: List[Dict] = []
        if summary_path.exists():
            try:
                with summary_path.open("r", encoding="utf-8", newline="") as stream:
                    rows.extend(csv.DictReader(stream))
            except (OSError, csv.Error):
                rows = []
        rows = [row for row in rows if row.get("run_id") != new_row.get("run_id")]
        rows.append(dict(new_row))
        rows.sort(key=lambda row: str(row.get("started_at", "")), reverse=True)
        _atomic_csv(summary_path, SUMMARY_FIELDS, rows)
        _atomic_text(self.root_dir / "index.html", _index_html(rows))
