import pytest

from rc_arm2_middleware.target_point_sampling import (
    TargetPointSample,
    average_valid_target_point_samples,
    select_target_point,
)


def test_averages_valid_samples_and_rejects_distance_outlier() -> None:
    samples = [
        TargetPointSample(0.300, 0.010, 0.200, 90.0),
        TargetPointSample(0.302, 0.011, 0.201, 91.0),
        TargetPointSample(0.301, 0.009, 0.199, 89.0),
        TargetPointSample(0.299, 0.010, 0.200, 90.0),
        TargetPointSample(0.303, 0.010, 0.201, 90.0),
        TargetPointSample(0.300, 0.012, 0.199, 91.0),
        TargetPointSample(0.301, 0.010, 0.200, 89.0),
        TargetPointSample(0.500, 0.010, 0.200, 90.0),
    ]

    result = average_valid_target_point_samples(
        samples,
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
    )

    assert result.target is not None
    assert result.accepted_count == 7
    assert result.rejected_count == 1
    assert result.target.x == pytest.approx(0.300857, abs=1e-6)
    assert result.target.y == pytest.approx(0.010286, abs=1e-6)
    assert result.target.z == pytest.approx(0.200000, abs=1e-6)
    assert result.target.target_spin_deg == pytest.approx(90.0, abs=1e-6)


def test_rejects_when_too_few_samples_remain_after_guard_filtering() -> None:
    samples = [
        TargetPointSample(0.300, 0.010, 0.200, 90.0),
        TargetPointSample(0.900, 0.010, 0.200, 90.0),
        TargetPointSample(0.301, 0.200, 0.200, 90.0),
    ]

    result = average_valid_target_point_samples(
        samples,
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
    )

    assert result.target is None
    assert result.accepted_count == 2
    assert result.rejected_count == 1
    assert "need at least 3 valid samples" in result.detail


def test_selects_visual_average_with_y_offset_before_fallback() -> None:
    samples = [
        TargetPointSample(0.20, 0.10, 0.30, 65.0),
        TargetPointSample(0.21, 0.10, 0.30, 65.0),
        TargetPointSample(0.19, 0.10, 0.30, 65.0),
    ]

    result = select_target_point(
        samples,
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
        target_y_offset=-0.06,
        fallback_xyz=(0.0, 0.0, 0.0),
        fallback_spin_deg=0.0,
        allow_fallback=True,
    )

    assert result.target is not None
    assert result.used_fallback is False
    assert result.target.y == pytest.approx(0.04)


def test_does_not_use_fallback_before_timeout() -> None:
    result = select_target_point(
        [],
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
        target_y_offset=-0.06,
        fallback_xyz=(0.0, 0.0, 0.0),
        fallback_spin_deg=65.0,
        allow_fallback=False,
    )

    assert result.target is None
    assert result.used_fallback is False


def test_uses_final_fallback_xyz_without_y_offset_at_timeout() -> None:
    result = select_target_point(
        [],
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
        target_y_offset=-0.06,
        fallback_xyz=(0.0, 0.0, 0.0),
        fallback_spin_deg=65.0,
        allow_fallback=True,
    )

    assert result.target == TargetPointSample(0.0, 0.0, 0.0, 65.0)
    assert result.used_fallback is True


def test_timeout_without_fallback_remains_unresolved() -> None:
    result = select_target_point(
        [],
        min_valid_count=3,
        max_sample_distance=0.03,
        max_abs_y=0.14,
        target_y_offset=-0.06,
        fallback_xyz=None,
        fallback_spin_deg=0.0,
        allow_fallback=True,
    )

    assert result.target is None
    assert result.used_fallback is False
