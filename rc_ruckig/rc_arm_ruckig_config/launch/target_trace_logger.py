#!/usr/bin/env python3
"""Subscribe trace events and append them to a JSONL log file."""

import argparse
import datetime as dt
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TargetTraceLogger(Node):
    def __init__(self, *, trace_event_topic: str, output_dir: str) -> None:
        super().__init__("rc_arm_target_trace_logger")
        base_dir = Path(output_dir).expanduser().resolve()
        base_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = base_dir / f"target_trace_{stamp}.jsonl"
        self._handle = self._path.open("a", encoding="utf-8")
        self.create_subscription(String, trace_event_topic, self._on_trace, 50)
        self.get_logger().info(f"writing trace log to {self._path}")

    def _on_trace(self, msg: String) -> None:
        self._handle.write(msg.data.rstrip("\n") + "\n")
        self._handle.flush()

    def destroy_node(self):
        try:
            self._handle.close()
        except Exception:
            pass
        return super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(description="trace event logger")
    parser.add_argument("--enabled", default="true")
    parser.add_argument("--trace-event-topic", default="/rc_arm_2/trace_event")
    parser.add_argument("--output-dir", default="/tmp/rc_arm_trace")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    enabled = str(args.enabled).strip().lower() in {"1", "true", "yes", "on"}
    if not enabled:
        return
    rclpy.init()
    node = TargetTraceLogger(trace_event_topic=args.trace_event_topic, output_dir=args.output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
