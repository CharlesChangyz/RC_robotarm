#!/usr/bin/env python3
"""Shared FK/IK helper for rc_arm_2 xyz + world-pitch control."""

from __future__ import annotations

import math
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
warnings.filterwarnings(
    "ignore",
    message=r"A NumPy version >=.* is required for this version of SciPy",
    category=UserWarning,
)
from scipy.optimize import least_squares


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_quat_xyzw(quat: Sequence[float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(v) * float(v) for v in quat))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(v) / norm for v in quat)


def quat_from_axis_angle(axis: str, angle_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * angle_rad
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (s, 0.0, 0.0, c)
    if axis == "y":
        return (0.0, s, 0.0, c)
    return (0.0, 0.0, s, c)


def extract_axis_angle_rad(axis: str, quat_xyzw: Sequence[float]) -> float:
    qx, qy, qz, qw = normalize_quat_xyzw(quat_xyzw)
    component = qx
    if axis == "y":
        component = qy
    elif axis == "z":
        component = qz
    return normalize_angle_rad(2.0 * math.atan2(component, qw))


def _local_name(tag: str) -> str:
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def _find_child(element: ET.Element, tag_name: str) -> Optional[ET.Element]:
    for child in list(element):
        if _local_name(child.tag) == tag_name:
            return child
    return None


def _parse_xyz(text: Optional[str]) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=float)
    values = [float(v) for v in text.split()]
    if len(values) != 3:
        raise ValueError("expected xyz triplet, got %r" % (text,))
    return np.array(values, dtype=float)


def _rotation_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        dtype=float,
    )


def _rotation_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        dtype=float,
    )


def _rotation_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _rpy_to_rotation(rpy: Sequence[float]) -> np.ndarray:
    r, p, y = (float(v) for v in rpy)
    return _rotation_z(y) @ _rotation_y(p) @ _rotation_x(r)


def _rotation_from_axis(axis_xyz: Sequence[float], angle: float) -> np.ndarray:
    axis = np.array(axis_xyz, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 1.0e-12:
        return np.eye(3, dtype=float)
    axis /= norm
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )


def _transform_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rpy_to_rotation(rpy)
    transform[:3, 3] = np.array(xyz, dtype=float)
    return transform


def _transform_from_rotation_translation(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array(translation, dtype=float)
    return transform


@dataclass(frozen=True)
class JointSpec:
    name: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Tuple[float, float, float]
    lower: float
    upper: float
    child: str


class RcArmWorldPitchKinematics:
    """Forward/inverse kinematics helper for the rc_arm_2 4DOF chain."""

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        joint_names: Optional[Iterable[str]] = None,
        j4_axis: str = "x",
    ) -> None:
        self._urdf_path = Path(urdf_path) if urdf_path else self.default_urdf_path()
        self._joint_names = list(joint_names or ("j1_joint", "j2_joint", "j3_joint", "j4_joint"))
        self._j4_axis = (j4_axis or "x").strip().lower()
        if self._j4_axis not in {"x", "y", "z"}:
            self._j4_axis = "x"

        self._joint_specs, self._fixed_end_effector = self._load_joint_specs(self._urdf_path)
        self._name_to_index = {name: idx for idx, name in enumerate(self._joint_names)}
        self.lower_limits = np.array([self._joint_specs[name].lower for name in self._joint_names], dtype=float)
        self.upper_limits = np.array([self._joint_specs[name].upper for name in self._joint_names], dtype=float)
        self.zero_joints = np.zeros(len(self._joint_names), dtype=float)
        self._zero_pitch_offset = self._raw_world_pitch(self.zero_joints)
        self._zero_position = self.forward_position(self.zero_joints)
        self._plane_y_constant = self._position_in_j1_frame(self.zero_joints, self._zero_position)[1]

    @staticmethod
    def default_urdf_path() -> Path:
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(get_package_share_directory("rc_arm_description"))
            return share_dir / "urdf" / "rc_arm_2" / "rc_arm_2.urdf.xacro"
        except Exception:
            return Path(__file__).resolve().parents[3] / "rc_moveit" / "rc_arm_description" / "urdf" / "rc_arm_2" / "rc_arm_2.urdf.xacro"

    @property
    def urdf_path(self) -> Path:
        return self._urdf_path

    def zero_home_pose(self) -> Tuple[float, float, float, float]:
        return (
            float(self._zero_position[0]),
            float(self._zero_position[1]),
            float(self._zero_position[2]),
            0.0,
        )

    def zero_home_joint_map(self) -> Dict[str, float]:
        return {name: 0.0 for name in self._joint_names}

    def joint_vector(self, joints: Sequence[float] | Dict[str, float]) -> np.ndarray:
        if isinstance(joints, dict):
            return np.array([float(joints[name]) for name in self._joint_names], dtype=float)
        values = list(joints)
        if len(values) != len(self._joint_names):
            raise ValueError("expected %d joints, got %d" % (len(self._joint_names), len(values)))
        return np.array([float(v) for v in values], dtype=float)

    def joint_map(self, joints: Sequence[float] | Dict[str, float]) -> Dict[str, float]:
        values = self.joint_vector(joints)
        return {name: float(values[idx]) for idx, name in enumerate(self._joint_names)}

    def within_limits(self, joints: Sequence[float] | Dict[str, float], margin: float = 1.0e-6) -> bool:
        values = self.joint_vector(joints)
        return bool(np.all(values >= self.lower_limits - margin) and np.all(values <= self.upper_limits + margin))

    def forward_transform(self, joints: Sequence[float] | Dict[str, float]) -> np.ndarray:
        values = self.joint_vector(joints)
        transform = np.eye(4, dtype=float)
        for idx, joint_name in enumerate(self._joint_names):
            spec = self._joint_specs[joint_name]
            origin_tf = _transform_from_xyz_rpy(spec.origin_xyz, spec.origin_rpy)
            rot_tf = np.eye(4, dtype=float)
            rot_tf[:3, :3] = _rotation_from_axis(spec.axis_xyz, float(values[idx]))
            transform = transform @ origin_tf @ rot_tf
        return transform @ self._fixed_end_effector

    def forward_position(self, joints: Sequence[float] | Dict[str, float]) -> Tuple[float, float, float]:
        transform = self.forward_transform(joints)
        position = transform[:3, 3]
        return (float(position[0]), float(position[1]), float(position[2]))

    def forward_world_pitch(self, joints: Sequence[float] | Dict[str, float]) -> float:
        values = self.joint_vector(joints)
        raw_pitch = self._raw_world_pitch(values)
        return normalize_angle_rad(raw_pitch - self._zero_pitch_offset)

    def world_pitch_from_quaternion(self, quat_xyzw: Sequence[float]) -> float:
        return extract_axis_angle_rad(self._j4_axis, quat_xyzw)

    def quaternion_from_world_pitch(self, pitch_rad: float) -> Tuple[float, float, float, float]:
        return quat_from_axis_angle(self._j4_axis, pitch_rad)

    def is_reachable(
        self,
        x: float,
        y: float,
        z: float,
        pitch_rad: float,
        seed_joints: Optional[Sequence[float] | Dict[str, float]] = None,
    ) -> bool:
        return self.solve_xyz_pitch(x, y, z, pitch_rad, seed_joints=seed_joints) is not None

    def solve_xyz_pitch(
        self,
        x: float,
        y: float,
        z: float,
        pitch_rad: float,
        seed_joints: Optional[Sequence[float] | Dict[str, float]] = None,
    ) -> Optional[Dict[str, float]]:
        target_xyz = np.array([float(x), float(y), float(z)], dtype=float)
        target_pitch = normalize_angle_rad(float(pitch_rad))
        candidate_seeds = self._build_seed_vectors(target_xyz, target_pitch, seed_joints)
        if not candidate_seeds:
            return None

        best = None
        best_cost = None
        best_limit_margin = None
        best_seed_metric = None
        seed_reference = self.joint_vector(seed_joints) if seed_joints is not None else self.zero_joints

        for seed in candidate_seeds:
            result = least_squares(
                lambda q: self._residual(q, target_xyz, target_pitch),
                x0=np.clip(seed, self.lower_limits, self.upper_limits),
                bounds=(self.lower_limits, self.upper_limits),
                method="trf",
                max_nfev=80,
                xtol=1.0e-8,
                ftol=1.0e-8,
                gtol=1.0e-8,
            )
            if not result.success:
                continue
            residual = self._residual(result.x, target_xyz, target_pitch)
            pos_error = float(np.linalg.norm(residual[:3]))
            pitch_error = abs(float(residual[3]))
            if pos_error > 2.0e-3 or pitch_error > math.radians(0.6):
                continue

            seed_metric = float(np.linalg.norm(result.x - seed_reference))
            limit_margin = float(np.min(np.minimum(result.x - self.lower_limits, self.upper_limits - result.x)))
            total_cost = pos_error + 0.05 * pitch_error
            if (
                best is None
                or total_cost < float(best_cost) - 1.0e-9
                or (
                    abs(total_cost - float(best_cost)) <= 1.0e-9
                    and (
                        best_limit_margin is None
                        or limit_margin > float(best_limit_margin) + 1.0e-9
                        or (
                            abs(limit_margin - float(best_limit_margin)) <= 1.0e-9
                            and seed_metric < float(best_seed_metric) - 1.0e-9
                        )
                    )
                )
            ):
                best = result.x.copy()
                best_cost = total_cost
                best_limit_margin = limit_margin
                best_seed_metric = seed_metric

        if best is None:
            return None
        return self.joint_map(best)

    def _raw_world_pitch(self, joints_vec: np.ndarray) -> float:
        transform = self.forward_transform(joints_vec)
        tool_direction_world = transform[:3, :3] @ np.array([1.0, 0.0, 0.0], dtype=float)
        tool_direction_plane = self._j1_cancel_rotation(float(joints_vec[0])) @ tool_direction_world
        return math.atan2(float(tool_direction_plane[2]), float(tool_direction_plane[0]))

    def _position_in_j1_frame(self, joints_vec: np.ndarray, world_position: Sequence[float]) -> np.ndarray:
        spec = self._joint_specs[self._joint_names[0]]
        origin = np.array(spec.origin_xyz, dtype=float)
        relative = np.array(world_position, dtype=float) - origin
        return self._j1_cancel_rotation(float(joints_vec[0])) @ relative

    def _j1_cancel_rotation(self, q1: float) -> np.ndarray:
        spec = self._joint_specs[self._joint_names[0]]
        axis_z = float(spec.axis_xyz[2])
        if abs(axis_z) <= 1.0e-9:
            return np.eye(3, dtype=float)
        origin_yaw = float(spec.origin_rpy[2])
        return _rotation_z(-(origin_yaw + axis_z * q1))

    def _residual(self, joints_vec: np.ndarray, target_xyz: np.ndarray, target_pitch: float) -> np.ndarray:
        position = np.array(self.forward_position(joints_vec), dtype=float)
        pitch = self.forward_world_pitch(joints_vec)
        return np.array(
            [
                position[0] - target_xyz[0],
                position[1] - target_xyz[1],
                position[2] - target_xyz[2],
                0.25 * normalize_angle_rad(pitch - target_pitch),
            ],
            dtype=float,
        )

    def _build_seed_vectors(
        self,
        target_xyz: np.ndarray,
        target_pitch: float,
        seed_joints: Optional[Sequence[float] | Dict[str, float]],
    ) -> List[np.ndarray]:
        seeds: List[np.ndarray] = []
        if seed_joints is not None:
            seeds.append(self.joint_vector(seed_joints))

        q1_guess = self._solve_q1_guess(float(target_xyz[0]), float(target_xyz[1]))
        zero = self.zero_joints.copy()
        if q1_guess is not None:
            zero[0] = q1_guess
        seeds.append(zero)

        if q1_guess is not None:
            mid = 0.5 * (self.lower_limits + self.upper_limits)
            mid[0] = q1_guess
            mid[3] = target_pitch
            seeds.append(mid)

            elbow_a = np.array([q1_guess, 0.4, 0.6, target_pitch], dtype=float)
            elbow_b = np.array([q1_guess, 1.2, 1.4, target_pitch], dtype=float)
            seeds.extend([elbow_a, elbow_b])

        deduped: List[np.ndarray] = []
        for seed in seeds:
            clipped = np.clip(seed, self.lower_limits, self.upper_limits)
            if any(np.linalg.norm(clipped - existing) <= 1.0e-6 for existing in deduped):
                continue
            deduped.append(clipped)
        return deduped

    def _solve_q1_guess(self, x: float, y: float) -> Optional[float]:
        spec = self._joint_specs[self._joint_names[0]]
        rel_x = float(x) - float(spec.origin_xyz[0])
        rel_y = float(y) - float(spec.origin_xyz[1])
        rho = math.hypot(rel_x, rel_y)
        const_y = float(self._plane_y_constant)
        if rho <= 1.0e-9 or rho + 1.0e-9 < abs(const_y):
            return None
        planar_x = math.sqrt(max(0.0, rho * rho - const_y * const_y))
        alpha = math.atan2(const_y, planar_x)
        beta = math.atan2(rel_y, rel_x)
        axis_z = float(spec.axis_xyz[2])
        if abs(axis_z) <= 1.0e-9:
            return None
        origin_yaw = float(spec.origin_rpy[2])
        guess = normalize_angle_rad((beta - alpha - origin_yaw) / axis_z)
        return float(np.clip(guess, self.lower_limits[0], self.upper_limits[0]))

    def _load_joint_specs(
        self,
        urdf_path: Path,
    ) -> Tuple[Dict[str, JointSpec], np.ndarray]:
        if not urdf_path.exists():
            raise FileNotFoundError("URDF/Xacro file not found: %s" % urdf_path)

        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
        joints_by_name: Dict[str, ET.Element] = {}
        for element in root.iter():
            if _local_name(element.tag) == "joint":
                name = (element.attrib.get("name") or "").strip()
                if name:
                    joints_by_name[name] = element

        specs: Dict[str, JointSpec] = {}
        for joint_name in self._joint_names:
            element = joints_by_name.get(joint_name)
            if element is None:
                raise KeyError("joint '%s' not found in %s" % (joint_name, urdf_path))

            origin_element = _find_child(element, "origin")
            axis_element = _find_child(element, "axis")
            limit_element = _find_child(element, "limit")
            child_element = _find_child(element, "child")

            origin_xyz = tuple(_parse_xyz(origin_element.attrib.get("xyz") if origin_element is not None else None))
            origin_rpy = tuple(_parse_xyz(origin_element.attrib.get("rpy") if origin_element is not None else None))
            axis_xyz = tuple(_parse_xyz(axis_element.attrib.get("xyz") if axis_element is not None else None))
            lower = float(limit_element.attrib.get("lower", "0.0")) if limit_element is not None else 0.0
            upper = float(limit_element.attrib.get("upper", "0.0")) if limit_element is not None else 0.0
            child = (child_element.attrib.get("link") if child_element is not None else "").strip()
            specs[joint_name] = JointSpec(
                name=joint_name,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis_xyz=axis_xyz,
                lower=lower,
                upper=upper,
                child=child,
            )

        fixed_eef = np.eye(4, dtype=float)
        for element in joints_by_name.values():
            if element.attrib.get("type") != "fixed":
                continue
            parent_element = _find_child(element, "parent")
            child_element = _find_child(element, "child")
            if parent_element is None or child_element is None:
                continue
            if (parent_element.attrib.get("link") or "").strip() != specs[self._joint_names[-1]].child:
                continue
            if (child_element.attrib.get("link") or "").strip() != "end_effector":
                continue
            origin_element = _find_child(element, "origin")
            fixed_eef = _transform_from_xyz_rpy(
                _parse_xyz(origin_element.attrib.get("xyz") if origin_element is not None else None),
                _parse_xyz(origin_element.attrib.get("rpy") if origin_element is not None else None),
            )
            break

        return specs, fixed_eef
