"""Surface estimation: six ranges in, a wall frame out.

Ranges are synthesised from a known plane, so the fit can be checked against
ground truth — including the cases that actually bite: sensors outside their
validity window (the ToF ceiling at 0.258 m), and a single wild outlier.
"""

import numpy as np
import pytest

from task_planner_fsm.wbc.surface import (
    SENSOR_XY,
    SurfaceEstimator,
    fit_wall_plane,
    plate_orientation_target,
    sweep_tangent,
)


def synth_ranges(normal, distance):
    """Ranges a plate would read facing a plane with the given normal/standoff.

    The plane is ``n . x = distance`` in plate coordinates (so ``distance`` is
    the perpendicular gap at the plate centre); sensor *i* sits at ``(x, y, 0)``
    and measures along +Z, so it reads ``(distance - n_x x - n_y y) / n_z``.
    """
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    return (distance - SENSOR_XY[:, 0] * n[0] - SENSOR_XY[:, 1] * n[1]) / n[2]


def test_a_parallel_plate_reads_the_standoff_and_a_z_normal():
    normal, distance, n_valid = fit_wall_plane(synth_ranges([0, 0, 1], 0.20))
    assert n_valid == 6
    np.testing.assert_allclose(normal, [0, 0, 1], atol=1e-9)
    assert distance == pytest.approx(0.20)


def test_a_tilted_plate_recovers_the_tilt_and_the_perpendicular_gap():
    truth = np.array([0.15, -0.08, 1.0])
    truth /= np.linalg.norm(truth)
    normal, distance, n_valid = fit_wall_plane(synth_ranges(truth, 0.22))
    # Tilted at a 0.22 m standoff, the far ToF corner is already past its
    # 0.258 m ceiling — five sensors still pin the plane exactly.
    assert n_valid == 5
    np.testing.assert_allclose(normal, truth, atol=1e-9)
    assert distance == pytest.approx(0.22, abs=1e-9)
    # The mean RANGE is longer than the perpendicular gap when tilted, which is
    # exactly why the fit is evaluated at the plate centre instead.
    assert np.mean(synth_ranges(truth, 0.22)) > distance


def test_out_of_range_tof_sensors_are_dropped_not_believed():
    # At a 0.5 m standoff the three ToF sensors are past their 0.258 m ceiling,
    # so the fit must rest on the ultrasonics alone.
    normal, distance, n_valid = fit_wall_plane(synth_ranges([0, 0, 1], 0.50))
    assert n_valid == 3
    np.testing.assert_allclose(normal, [0, 0, 1], atol=1e-9)
    assert distance == pytest.approx(0.50)


def test_too_few_valid_sensors_is_reported_rather_than_guessed():
    ranges = synth_ranges([0, 0, 1], 0.20)
    ranges[:4] = np.nan
    normal, distance, n_valid = fit_wall_plane(ranges)
    assert normal is None and distance is None and n_valid == 2


def test_one_wild_reading_does_not_tip_the_plane():
    truth = np.array([0.1, 0.0, 1.0])
    truth /= np.linalg.norm(truth)
    ranges = synth_ranges(truth, 0.20)
    clean, _, _ = fit_wall_plane(ranges)
    ranges[1] += 0.08                      # one ultrasonic reading 8 cm long
    robust, _, _ = fit_wall_plane(ranges)
    naive = np.linalg.lstsq(
        np.column_stack((SENSOR_XY[:, 0], SENSOR_XY[:, 1], np.ones(6))), ranges, rcond=None)[0]
    naive_normal = np.array([-naive[0], -naive[1], 1.0])
    naive_normal /= np.linalg.norm(naive_normal)
    assert np.linalg.norm(robust - clean) < np.linalg.norm(naive_normal - clean)


def test_the_estimator_filters_toward_the_truth():
    estimator = SurfaceEstimator(ema_alpha=0.5)
    truth = np.array([0.2, 0.0, 1.0])
    truth /= np.linalg.norm(truth)
    ranges = synth_ranges(truth, 0.20)
    for _ in range(20):
        assert estimator.update(ranges)
    np.testing.assert_allclose(estimator.normal_plate, truth, atol=1e-6)
    assert estimator.distance == pytest.approx(0.20, abs=1e-9)
    assert estimator.tilt() == pytest.approx(np.arccos(truth[2]), abs=1e-6)


def test_the_estimator_refuses_an_unusable_frame():
    estimator = SurfaceEstimator()
    assert not estimator.update([np.nan] * 6)
    assert estimator.distance is None


def test_the_sweep_tangent_lies_in_the_sensed_surface():
    m_hat = np.array([1.0, 0.2, 0.0])
    m_hat /= np.linalg.norm(m_hat)
    t_hat = sweep_tangent(m_hat, np.array([0.0, 1.0, 0.0]))
    assert np.dot(t_hat, m_hat) == pytest.approx(0.0, abs=1e-12)
    assert np.linalg.norm(t_hat) == pytest.approx(1.0)
    # It still points the way the segment does.
    assert np.dot(t_hat, [0.0, 1.0, 0.0]) > 0.0


def test_a_scan_direction_straight_into_the_wall_is_rejected():
    assert sweep_tangent(np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])) is None


def test_the_orientation_target_faces_the_wall_and_stands_upright():
    m_hat = np.array([1.0, 0.3, -0.1])
    m_hat /= np.linalg.norm(m_hat)
    R = plate_orientation_target(m_hat)
    np.testing.assert_allclose(R[:, 2], m_hat, atol=1e-12)          # plate +Z into the wall
    assert R[2, 1] > 0.0                                            # plate +Y points up
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)      # orthonormal
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_a_vertical_surface_normal_has_no_upright_solution():
    assert plate_orientation_target(np.array([0.0, 0.0, 1.0])) is None
