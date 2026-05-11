#!/usr/bin/env python3
"""Small helpers for JSON trace events used by the target executor."""

import json
from typing import Dict, Optional

from std_msgs.msg import String


def trace_id_from_stamp(stamp) -> str:
    return f"{int(stamp.sec)}-{int(stamp.nanosec)}"


def target_fields_from_values(x: float, y: float, z: float, *, j4_rad: Optional[float] = None) -> Dict[str, float]:
    data = {"x": float(x), "y": float(y), "z": float(z)}
    if j4_rad is not None:
        data["j4_rad"] = float(j4_rad)
    return data


def build_trace_event(
    *,
    trace_id: str,
    event: str,
    source: str,
    node: str,
    event_ns: int,
    target_fields: Optional[Dict[str, float]] = None,
    extra: Optional[Dict[str, object]] = None,
) -> str:
    payload = {
        "trace_id": str(trace_id),
        "event": str(event),
        "source": str(source),
        "node": str(node),
        "event_ns": int(event_ns),
    }
    if target_fields:
        payload["target"] = dict(target_fields)
    if extra:
        payload["extra"] = dict(extra)
    return json.dumps(payload, ensure_ascii=True, sort_keys=True)


def publish_trace_event(publisher, payload: str) -> None:
    msg = String()
    msg.data = payload
    publisher.publish(msg)
