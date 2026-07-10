"""Helpers for filtering and averaging target point samples."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Sequence


@dataclass(frozen=True)
class TargetPointSample:
    x: float
    y: float
    z: float
    target_spin_deg: float


@dataclass(frozen=True)
class TargetPointSamplingResult:
    target: TargetPointSample | None
    accepted_count: int
    rejected_count: int
    detail: str


@dataclass(frozen=True)
class TargetPointSelectionResult:
    target: TargetPointSample | None
    used_fallback: bool
    sampling: TargetPointSamplingResult


def average_valid_target_point_samples(
    samples: Sequence[TargetPointSample],
    *,
    min_valid_count: int,
    max_sample_distance: float,
    max_x: float,
    max_abs_y: float,
) -> TargetPointSamplingResult:
    guard_valid = [
        sample
        for sample in samples
        if _sample_is_finite(sample)
        and float(sample.x) <= float(max_x)
        and abs(float(sample.y)) <= float(max_abs_y)
    ]
    rejected_count = len(samples) - len(guard_valid)

    if len(guard_valid) < min_valid_count:
        return TargetPointSamplingResult(
            target=None,
            accepted_count=len(guard_valid),
            rejected_count=rejected_count,
            detail=(
                "need at least %d valid samples, got %d after guard filtering"
                % (min_valid_count, len(guard_valid))
            ),
        )

    center = TargetPointSample(
        x=float(median(sample.x for sample in guard_valid)),
        y=float(median(sample.y for sample in guard_valid)),
        z=float(median(sample.z for sample in guard_valid)),
        target_spin_deg=float(median(sample.target_spin_deg for sample in guard_valid)),
    )
    distance_valid = [
        sample
        for sample in guard_valid
        if _position_distance(sample, center) <= float(max_sample_distance)
    ]
    rejected_count += len(guard_valid) - len(distance_valid)

    if len(distance_valid) < min_valid_count:
        return TargetPointSamplingResult(
            target=None,
            accepted_count=len(distance_valid),
            rejected_count=rejected_count,
            detail=(
                "need at least %d valid samples, got %d after distance filtering"
                % (min_valid_count, len(distance_valid))
            ),
        )

    target = TargetPointSample(
        x=sum(sample.x for sample in distance_valid) / len(distance_valid),
        y=sum(sample.y for sample in distance_valid) / len(distance_valid),
        z=sum(sample.z for sample in distance_valid) / len(distance_valid),
        target_spin_deg=(
            sum(sample.target_spin_deg for sample in distance_valid)
            / len(distance_valid)
        ),
    )
    return TargetPointSamplingResult(
        target=target,
        accepted_count=len(distance_valid),
        rejected_count=rejected_count,
        detail=(
            "averaged %d valid target point samples, rejected %d"
            % (len(distance_valid), rejected_count)
        ),
    )


def select_target_point(
    samples: Sequence[TargetPointSample],
    *,
    min_valid_count: int,
    max_sample_distance: float,
    max_x: float,
    max_abs_y: float,
    target_y_offset: float,
    fallback_xyz: tuple[float, float, float] | None,
    fallback_spin_deg: float,
    allow_fallback: bool,
) -> TargetPointSelectionResult:
    sampling = average_valid_target_point_samples(
        samples,
        min_valid_count=min_valid_count,
        max_sample_distance=max_sample_distance,
        max_x=max_x,
        max_abs_y=max_abs_y,
    )
    if sampling.target is not None:
        visual = sampling.target
        return TargetPointSelectionResult(
            target=TargetPointSample(
                x=visual.x,
                y=visual.y + float(target_y_offset),
                z=visual.z,
                target_spin_deg=visual.target_spin_deg,
            ),
            used_fallback=False,
            sampling=sampling,
        )

    if allow_fallback and fallback_xyz is not None:
        return TargetPointSelectionResult(
            target=TargetPointSample(
                x=float(fallback_xyz[0]),
                y=float(fallback_xyz[1]),
                z=float(fallback_xyz[2]),
                target_spin_deg=float(fallback_spin_deg),
            ),
            used_fallback=True,
            sampling=sampling,
        )

    return TargetPointSelectionResult(
        target=None,
        used_fallback=False,
        sampling=sampling,
    )


def _sample_is_finite(sample: TargetPointSample) -> bool:
    return all(
        math.isfinite(value)
        for value in (sample.x, sample.y, sample.z, sample.target_spin_deg)
    )


def _position_distance(a: TargetPointSample, b: TargetPointSample) -> float:
    return math.sqrt(
        (float(a.x) - float(b.x)) ** 2
        + (float(a.y) - float(b.y)) ** 2
        + (float(a.z) - float(b.z)) ** 2
    )
