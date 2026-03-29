#!/usr/bin/env python3
"""Interactive TF target publisher for rc_arm_2."""

import argparse
import math
import threading
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


@dataclass
class TargetState:
    x: float
    y: float
    z: float
    j4_rad: float


def normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")


def quat_from_axis_angle(axis: str, angle_rad: float):
    half = 0.5 * angle_rad
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (s, 0.0, 0.0, c)
    if axis == "y":
        return (0.0, s, 0.0, c)
    return (0.0, 0.0, s, c)


class InteractiveTfPublisher(Node):
    def __init__(
        self,
        tf_topic: str,
        parent_frame: str,
        child_frame: str,
        publish_rate: float,
        print_publish: bool,
        j4_axis: str,
        input_in_radians: bool,
        initial_xyz,
        initial_j4,
    ) -> None:
        super().__init__("rc_arm_tf_target_cli_publisher")

        self._tf_topic = tf_topic
        self._parent_frame = normalize_frame_id(parent_frame)
        self._child_frame = normalize_frame_id(child_frame)
        self._print_publish = bool(print_publish)
        self._j4_axis = j4_axis
        self._input_in_radians = input_in_radians

        if not self._parent_frame or not self._child_frame:
            raise ValueError("parent/child frame cannot be empty")

        init_j4_rad = float(initial_j4) if input_in_radians else math.radians(float(initial_j4))
        self._state = TargetState(
            x=float(initial_xyz[0]),
            y=float(initial_xyz[1]),
            z=float(initial_xyz[2]),
            j4_rad=init_j4_rad,
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._tf_pub = self.create_publisher(TFMessage, self._tf_topic, 20)

        rate = max(float(publish_rate), 1.0)
        self._timer = self.create_timer(1.0 / rate, self._publish_target)

        self.get_logger().info(
            "Interactive TF target publisher started: tf=%s %s->%s rate=%.1fHz, j4_axis=%s, unit=%s"
            % (
                self._tf_topic,
                self._parent_frame,
                self._child_frame,
                rate,
                self._j4_axis,
                "rad" if self._input_in_radians else "deg",
            )
        )
        self._print_state(prefix="Initial target")
        self._print_help()

        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    @property
    def shutdown_requested(self) -> bool:
        return self._stop_event.is_set()

    def _print_help(self) -> None:
        print("\nInput format:")
        print("  x y z j4      -> update position and j4")
        print("  x y z         -> update position only")
        print("  j4 value      -> update j4 only")
        print("  show          -> print current target")
        print("  q             -> quit")

    def _parse_number_list(self, text: str):
        normalized = text.replace(",", " ").strip()
        if not normalized:
            return []
        return [float(x) for x in normalized.split()]

    def _input_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = input("target> ").strip()
            except EOFError:
                self._stop_event.set()
                break
            except KeyboardInterrupt:
                self._stop_event.set()
                break

            if not line:
                continue

            low = line.lower()
            if low in {"q", "quit", "exit"}:
                self._stop_event.set()
                break
            if low == "show":
                self._print_state()
                continue

            try:
                self._handle_input(line)
            except Exception as exc:
                print(f"Invalid input: {exc}")
                self._print_help()

    def _handle_input(self, line: str) -> None:
        tokens = line.replace(",", " ").split()
        if len(tokens) == 2 and tokens[0].lower() == "j4":
            j4_raw = float(tokens[1])
            j4_rad = j4_raw if self._input_in_radians else math.radians(j4_raw)
            with self._lock:
                self._state.j4_rad = j4_rad
            self._print_state()
            return

        values = self._parse_number_list(line)
        if len(values) == 4:
            j4_raw = values[3]
            j4_rad = j4_raw if self._input_in_radians else math.radians(j4_raw)
            with self._lock:
                self._state.x = values[0]
                self._state.y = values[1]
                self._state.z = values[2]
                self._state.j4_rad = j4_rad
            self._print_state()
            return

        if len(values) == 3:
            with self._lock:
                self._state.x = values[0]
                self._state.y = values[1]
                self._state.z = values[2]
            self._print_state()
            return

        raise ValueError("expected: 'x y z j4' or 'x y z' or 'j4 value'")

    def _print_state(self, prefix: str = "Current target") -> None:
        with self._lock:
            st = TargetState(self._state.x, self._state.y, self._state.z, self._state.j4_rad)
        j4_deg = math.degrees(st.j4_rad)
        print(
            "%s: x=%.4f y=%.4f z=%.4f j4=%.3f deg (%.4f rad)"
            % (prefix, st.x, st.y, st.z, j4_deg, st.j4_rad)
        )

    def _publish_target(self) -> None:
        with self._lock:
            st = TargetState(self._state.x, self._state.y, self._state.z, self._state.j4_rad)

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_axis_angle(self._j4_axis, st.j4_rad)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._parent_frame
        tf_msg.child_frame_id = self._child_frame
        tf_msg.transform.translation.x = st.x
        tf_msg.transform.translation.y = st.y
        tf_msg.transform.translation.z = st.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self._tf_pub.publish(TFMessage(transforms=[tf_msg]))
        if self._print_publish:
            self._print_state(prefix="Published target")


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive TF target publisher")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--parent-frame", default="world")
    parser.add_argument("--child-frame", default="rc_arm_2_target")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--print-publish", action="store_true", help="Print every publish (verbose)")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--radians", action="store_true", help="Treat input j4 as radians (default: degrees)")
    parser.add_argument("--init-x", type=float, default=0.099)
    parser.add_argument("--init-y", type=float, default=0.026)
    parser.add_argument("--init-z", type=float, default=0.242)
    parser.add_argument("--init-j4", type=float, default=0.0, help="deg by default, rad with --radians")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = InteractiveTfPublisher(
        tf_topic=args.tf_topic,
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
        publish_rate=args.rate,
        print_publish=args.print_publish,
        j4_axis=args.j4_axis,
        input_in_radians=args.radians,
        initial_xyz=(args.init_x, args.init_y, args.init_z),
        initial_j4=args.init_j4,
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
