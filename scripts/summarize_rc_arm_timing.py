#!/usr/bin/env python3
"""Aggregate rc_arm timing JSONL files into a CSV latency report."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_TIMING_DIR = REPO_ROOT / "logs" / "rc_arm_timing"

EVENT_FILES = (
    "gui.events.jsonl",
    "bridge.events.jsonl",
    "executor.events.jsonl",
    "controller.events.jsonl",
)

STAGE_COLUMNS = (
    ("gui_send_click", "gui_tf_published", "gui_click_to_tf_ms"),
    ("gui_tf_published", "bridge_target_pose_published", "tf_to_bridge_ms"),
    ("bridge_target_pose_published", "executor_target_rx", "bridge_to_executor_rx_ms"),
    ("executor_target_rx", "executor_solve_ok", "executor_rx_to_solve_ms"),
    ("executor_solve_ok", "executor_goal_send", "solve_to_goal_send_ms"),
    ("executor_goal_send", "executor_goal_accepted", "goal_send_to_goal_accepted_ms"),
    ("executor_goal_accepted", "controller_goal_accepted", "executor_to_controller_accept_ms"),
    ("controller_goal_accepted", "controller_goal_finished", "controller_exec_ms"),
    ("gui_send_click", "controller_goal_finished", "total_ms"),
)

FAIL_EVENTS = {
    "executor_solve_fail",
    "executor_goal_rejected",
    "executor_goal_send_exception",
    "executor_exec_fail",
    "executor_exec_result_exception",
    "controller_goal_rejected",
    "controller_goal_invalid",
    "controller_goal_canceled",
    "controller_goal_preempted",
    "controller_goal_deactivated",
}


def resolve_timing_dir(raw: str) -> Path:
    text = (raw or "").strip()
    if text:
        return Path(text).expanduser()
    return DEFAULT_TIMING_DIR


def load_records(timing_dir: Path) -> Dict[str, List[dict]]:
    traces: Dict[str, List[dict]] = {}
    for filename in EVENT_FILES:
        path = timing_dir / filename
        if not path.is_file():
            continue
        with path.open("r", encoding="utf-8") as fp:
            for line_no, line in enumerate(fp, start=1):
                text = line.strip()
                if not text:
                    continue
                try:
                    record = json.loads(text)
                except json.JSONDecodeError as exc:
                    print(f"skip invalid JSON {path}:{line_no}: {exc}", file=sys.stderr)
                    continue
                trace_id = str(record.get("trace_id", "")).strip()
                if not trace_id:
                    continue
                traces.setdefault(trace_id, []).append(record)
    return traces


def first_event_map(records: Iterable[dict]) -> Dict[str, dict]:
    selected: Dict[str, dict] = {}
    ordered = sorted(records, key=lambda item: (int(item.get("wall_time_ns", 0)), str(item.get("event", ""))))
    for record in ordered:
        event = str(record.get("event", "")).strip()
        if event and event not in selected:
            selected[event] = record
    return selected


def duration_ms(start: Optional[dict], end: Optional[dict]) -> str:
    if not start or not end:
        return ""
    try:
        delta_ns = int(end["wall_time_ns"]) - int(start["wall_time_ns"])
    except (KeyError, TypeError, ValueError):
        return ""
    return f"{delta_ns / 1_000_000.0:.3f}"


def derive_status(event_map: Dict[str, dict]) -> str:
    for event in FAIL_EVENTS:
        record = event_map.get(event)
        if record is not None and str(record.get("status", "ok")).lower() != "ok":
            return "failed"
    if "controller_goal_finished" in event_map:
        return "success"
    return "failed"


def extract_target(event_map: Dict[str, dict]) -> dict:
    for event_name in (
        "gui_tf_published",
        "bridge_target_pose_published",
        "executor_target_rx",
    ):
        record = event_map.get(event_name)
        if record and isinstance(record.get("target"), dict):
            return record["target"]
    return {}


def extract_failure_event(event_map: Dict[str, dict]) -> str:
    for record in sorted(event_map.values(), key=lambda item: int(item.get("wall_time_ns", 0))):
        event = str(record.get("event", "")).strip()
        status = str(record.get("status", "ok")).lower()
        if event in FAIL_EVENTS and status != "ok":
            return event
    return ""


def build_rows(traces: Dict[str, List[dict]]) -> List[dict]:
    rows: List[dict] = []
    for trace_id, records in sorted(traces.items(), key=lambda item: int(item[0])):
        event_map = first_event_map(records)
        target = extract_target(event_map)
        row = {
            "trace_id": trace_id,
            "status": derive_status(event_map),
            "failure_event": extract_failure_event(event_map),
            "target_x": target.get("x", ""),
            "target_y": target.get("y", ""),
            "target_z": target.get("z", ""),
            "target_j4_rad": target.get("j4_rad", ""),
        }
        for start_name, end_name, column in STAGE_COLUMNS:
            row[column] = duration_ms(event_map.get(start_name), event_map.get(end_name))
        rows.append(row)
    return rows


def write_csv(rows: List[dict], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "trace_id",
        "status",
        "failure_event",
        "target_x",
        "target_y",
        "target_z",
        "target_j4_rad",
    ] + [column for _, _, column in STAGE_COLUMNS]
    with output_path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize rc_arm timing JSONL traces")
    parser.add_argument("--timing-dir", default=str(DEFAULT_TIMING_DIR))
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    timing_dir = resolve_timing_dir(args.timing_dir)
    output_path = Path(args.output).expanduser() if args.output else timing_dir / "summary.csv"

    traces = load_records(timing_dir)
    rows = build_rows(traces)
    write_csv(rows, output_path)
    print(f"wrote {len(rows)} traces to {output_path}")


if __name__ == "__main__":
    main()
