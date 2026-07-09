#!/usr/bin/env python3
"""Resolve and preflight a RealSense device for the arm KFS camera process."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Iterable, List

import pyrealsense2 as rs


@dataclass
class DeviceInfo:
    name: str
    serial: str
    product_id: str
    firmware: str
    usb_type: str


def _get_info(device, field) -> str:
    try:
        return device.get_info(field)
    except Exception:
        return ""


def _log(message: str) -> None:
    print(f"[rs-resolve] {message}", file=sys.stderr, flush=True)


def _device_summary(device: DeviceInfo) -> str:
    return (
        f"serial={device.serial or '<empty>'} name={device.name or '<unknown>'} "
        f"product_id={device.product_id or '<unknown>'} usb={device.usb_type or '<unknown>'} "
        f"firmware={device.firmware or '<unknown>'}"
    )


def list_devices() -> List[DeviceInfo]:
    ctx = rs.context()
    devices: List[DeviceInfo] = []
    for device in ctx.query_devices():
        name = _get_info(device, rs.camera_info.name)
        if name.strip().lower() == "platform camera":
            continue
        devices.append(
            DeviceInfo(
                name=name,
                serial=_get_info(device, rs.camera_info.serial_number),
                product_id=_get_info(device, rs.camera_info.product_id),
                firmware=_get_info(device, rs.camera_info.firmware_version),
                usb_type=_get_info(device, rs.camera_info.usb_type_descriptor),
            )
        )
    return devices


def _d435_candidates(devices: Iterable[DeviceInfo]) -> List[DeviceInfo]:
    result = []
    for device in devices:
        name = device.name.lower()
        product_id = device.product_id.upper()
        if "d435" in name or product_id == "0B3A":
            result.append(device)
    return result


def choose_device(devices: List[DeviceInfo], requested_serial: str) -> DeviceInfo:
    requested = (requested_serial or "auto").strip()
    if not requested:
        requested = "auto"

    _log(f"visible RealSense devices: {len(devices)}")
    for device in devices:
        _log(f"  {_device_summary(device)}")

    if requested.lower() not in {"auto", "any"}:
        for device in devices:
            if device.serial == requested:
                _log(f"using requested serial: {requested}")
                return device
        available = ", ".join(device.serial or "<empty>" for device in devices) or "<none>"
        raise RuntimeError(f"requested RealSense serial {requested!r} not found; available: {available}")

    candidates = _d435_candidates(devices)
    if not candidates:
        candidates = devices

    if not candidates:
        raise RuntimeError("no RealSense devices are visible to pyrealsense2")
    if len(candidates) > 1:
        available = ", ".join(_device_summary(device) for device in candidates)
        raise RuntimeError(f"multiple RealSense devices found; set REALSENSE_SERIAL explicitly: {available}")

    _log(f"auto selected: {_device_summary(candidates[0])}")
    return candidates[0]


def preflight(device: DeviceInfo, width: int, height: int, fps: int, frame_timeout_ms: int) -> None:
    if not device.serial:
        raise RuntimeError("selected RealSense device has an empty serial")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device.serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    started = False
    try:
        _log(
            f"preflight start serial={device.serial} "
            f"color/depth={width}x{height}@{fps} timeout_ms={frame_timeout_ms}"
        )
        pipeline.start(config)
        started = True
        frames = pipeline.wait_for_frames(frame_timeout_ms)
        if not frames.get_color_frame() or not frames.get_depth_frame():
            raise RuntimeError("started pipeline but did not receive both color and depth frames")
        _log(f"preflight OK serial={device.serial}")
    finally:
        if started:
            pipeline.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resolve and preflight the arm RealSense camera.")
    parser.add_argument("--serial", default="auto", help="RealSense serial, auto, or any")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frame-timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--no-preflight",
        action="store_true",
        help="Only resolve the serial; do not start the RealSense stream.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        device = choose_device(list_devices(), args.serial)
        if not args.no_preflight:
            preflight(device, args.width, args.height, args.fps, args.frame_timeout_ms)
        print(device.serial, flush=True)
        return 0
    except Exception as exc:
        _log(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
