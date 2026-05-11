#!/usr/bin/env python3
"""Shared JSONL timing logger for the rc_arm target execution chain."""

import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TIMING_DIR = REPO_ROOT / "logs" / "rc_arm_timing"


def resolve_timing_dir() -> Path:
    configured = os.environ.get("RC_ARM_TIMING_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    return DEFAULT_TIMING_DIR


def ros_stamp_to_ns(stamp: object) -> Optional[int]:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    try:
        return int(sec) * 1_000_000_000 + int(nanosec)
    except (TypeError, ValueError):
        return None


def trace_id_from_ros_stamp(stamp: object) -> Optional[str]:
    value = ros_stamp_to_ns(stamp)
    if value is None:
        return None
    return str(value)


class JsonlTimingLogger:
    def __init__(self, component: str) -> None:
        self._component = str(component).strip() or "unknown"
        self._dir = resolve_timing_dir()
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path = self._dir / f"{self._component}.events.jsonl"
        self._lock = threading.Lock()
        self._fp = self._path.open("a", encoding="utf-8", buffering=1)

    @property
    def path(self) -> Path:
        return self._path

    def log_event(
        self,
        event: str,
        trace_id: Optional[str],
        *,
        ros_stamp: object = None,
        wall_time_ns: Optional[int] = None,
        status: str = "ok",
        **extra: Any,
    ) -> None:
        record = {
            "trace_id": str(trace_id) if trace_id is not None else "",
            "component": self._component,
            "event": str(event),
            "wall_time_ns": int(wall_time_ns) if wall_time_ns is not None else time.time_ns(),
            "status": str(status),
        }
        ros_stamp_ns = ros_stamp_to_ns(ros_stamp)
        if ros_stamp_ns is not None:
            record["ros_stamp_ns"] = ros_stamp_ns
        for key, value in extra.items():
            if value is not None:
                record[key] = value

        line = json.dumps(record, ensure_ascii=True, separators=(",", ":"))
        with self._lock:
            self._fp.write(line + "\n")
            self._fp.flush()

    def close(self) -> None:
        with self._lock:
            if not self._fp.closed:
                self._fp.close()
