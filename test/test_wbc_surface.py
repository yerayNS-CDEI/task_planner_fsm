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
    estimator = SurfaceEstimator(tau=0.5)
    truth = np.array([0.2, 0.0, 1.0])
    truth /= np.linalg.norm(truth)
    ranges = synth_ranges(truth, 0.20)
    for k in range(20):
        assert estimator.update(ranges, stamp=0.25 * k)
    np.testing.assert_allclose(estimator.normal_plate, truth, atol=1e-6)
    np.testing.assert_allclose(estimator.normal_world, truth, atol=1e-6)   # identity plate
    assert estimator.distance == pytest.approx(0.20, abs=1e-9)
    assert estimator.tilt() == pytest.approx(np.arccos(truth[2]), abs=1e-6)


def test_the_estimator_refuses_an_unusable_frame():
    estimator = SurfaceEstimator()
    assert not estimator.update([np.nan] * 6, stamp=0.0)
    assert estimator.distance is None


def test_the_first_frame_is_taken_whole_and_the_time_constant_is_in_seconds():
    """Fold a frame at 0 s, then a different one at tau seconds later: the
    estimate has moved 1 - 1/e of the way, whatever the loop rate."""
    estimator = SurfaceEstimator(tau=0.5)
    n0 = np.array([0.0, 0.0, 1.0])
    n1 = np.array([0.1, 0.0, 1.0]) / np.linalg.norm([0.1, 0.0, 1.0])
    assert estimator.update(synth_ranges(n0, 0.2), stamp=10.0)
    np.testing.assert_allclose(estimator.normal_world, n0, atol=1e-9)
    assert estimator.update(synth_ranges(n1, 0.2), stamp=10.5)
    expected = (1 - np.exp(-1)) * n1 + np.exp(-1) * n0
    expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(estimator.normal_world, expected, atol=1e-6)


def test_a_frame_is_folded_in_once_however_often_the_loop_asks():
    """The control loop runs many cycles per range frame. Re-presenting the
    same stamp must not advance the filter — that is what made its strength a
    function of the loop rate (99% transparent at 50 Hz on 4 Hz frames)."""
    estimator = SurfaceEstimator(tau=0.5)
    n0 = np.array([0.0, 0.0, 1.0])
    n1 = np.array([0.1, 0.0, 1.0]) / np.linalg.norm([0.1, 0.0, 1.0])
    estimator.update(synth_ranges(n0, 0.2), stamp=0.0)
    for _ in range(12):
        assert estimator.update(synth_ranges(n1, 0.2), stamp=0.25)
    once = SurfaceEstimator(tau=0.5)
    once.update(synth_ranges(n0, 0.2), stamp=0.0)
    once.update(synth_ranges(n1, 0.2), stamp=0.25)
    np.testing.assert_allclose(estimator.normal_world, once.normal_world, atol=1e-12)
    assert np.dot(estimator.normal_world, n1) < 0.9999       # nowhere near fully converged


def test_the_filter_lives_in_the_world_so_the_plate_can_turn_without_lag():
    """Same wall, plate rotated between frames: the fit in plate coordinates
    changes, the world normal does not, and the filter must report the same
    world normal with no transient. A plate-frame filter would have blended
    two plate-frame normals and produced a wall that is not there."""
    wall = np.array([0.05, 0.0, 1.0]) / np.linalg.norm([0.05, 0.0, 1.0])
    estimator = SurfaceEstimator(tau=1.0)
    for k, angle in enumerate([0.0, 0.05, -0.05, 0.10]):
        c, s_ = np.cos(angle), np.sin(angle)
        R_plate = np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])   # plate yawed about y
        n_plate = R_plate.T @ wall                                  # what the sensors see
        assert estimator.update(synth_ranges(n_plate, 0.2), R_plate, stamp=0.25 * k)
        np.testing.assert_allclose(estimator.normal_world, wall, atol=1e-9)
        np.testing.assert_allclose(estimator.normal_in(R_plate), n_plate, atol=1e-9)


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


# ----------------------------------------------------------------------
# Against the robot: the 2026-09-17 calibration bag, replayed
# ----------------------------------------------------------------------
#
# test/fixtures/plate_ranges_2026_09_17.json holds, for 17 poses the arm
# held still in front of a concrete wall, the mean of the six RAW published
# ranges and the wall's true normal and distance in the plate frame from the
# arm's forward kinematics (the pendant TCP is the GPR contact point, and it
# was constrained to lie on the wall at the two contact poses). It also holds
# the 104 raw range frames from the approach that ended in the 2026-09-15
# 31 N overload.
#
# The reader now subtracts a constant offset per sensor before publishing
# (arm_control sensors/plate_calibration.py). These tests apply the same
# offsets to the raw fixture and check that THIS fit, with THESE sigmas, then
# recovers what the arm measured — and that without the offsets it does not.
# If the offsets ever change, the fixture's copy must change with them.

import json
import pathlib


@pytest.fixture(scope="module")
def field():
    path = pathlib.Path(__file__).parent / "fixtures" / "plate_ranges_2026_09_17.json"
    return json.loads(path.read_text())


def _tilt_deg(normal):
    return np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))


def _angle_deg(a, b):
    return np.degrees(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1.0, 1.0)))


def test_calibrated_ranges_fit_the_wall_the_arm_measured(field):
    """Corrected ranges through the fit land on the FK plane, at every pose:
    normal within 2 deg, sensor-plane distance within 8 mm."""
    offset = np.array(field["range_offset_m"])
    for pose in field["poses"]:
        normal, distance, n_valid = fit_wall_plane(np.array(pose["raw_mean"]) - offset)
        assert n_valid == 6, pose["t_start"]
        err = _angle_deg(normal, pose["fk_normal_plate"])
        assert err < 2.0, f"pose {pose['t_start']}s: normal {err:.2f} deg off the FK"
        assert abs(distance - pose["fk_sensor_plane_distance"]) < 0.008, (
            f"pose {pose['t_start']}s: {distance:.4f} vs FK {pose['fk_sensor_plane_distance']:.4f}")


def test_uncalibrated_ranges_fit_a_wall_that_is_not_there(field):
    """Without the offsets the same fit is 3-5 deg wrong — the offset pattern
    (U2 short, S2 long) cancels real tilt. This is the failure the reader's
    calibration exists to remove; if it passes on raw ranges, the fixture no
    longer describes the hardware."""
    errors = [_angle_deg(fit_wall_plane(np.array(p["raw_mean"]))[0], p["fk_normal_plate"])
              for p in field["poses"]]
    assert max(errors) > 3.0
    assert sum(e > 2.0 for e in errors) >= 6


def test_the_approach_that_overloaded_was_seven_degrees_off(field):
    """The 2026-09-15 31 N overload: the controller read the plate ~1 deg from
    parallel on raw ranges while it was ~7-8 deg off. Corrected ranges show
    the tilt that put one corner into the wall first."""
    offset = np.array(field["range_offset_m"])
    frames = np.array(field["failure_window_raw"])
    raw = np.array([_tilt_deg(fit_wall_plane(f)[0]) for f in frames])
    corrected = np.array([_tilt_deg(fit_wall_plane(f - offset)[0]) for f in frames])
    assert raw.mean() < 2.0
    assert 6.0 < corrected.mean() < 9.5


def test_corrected_tilt_jitter_fits_under_a_one_degree_deadband(field):
    """Frame-to-frame jitter of the corrected, ToF-weighted tilt on the same
    approach: p95 under 1.2 deg, so a ~1 deg soft deadband on the alignment
    task rejects the sensor noise without hiding a real tilt."""
    offset = np.array(field["range_offset_m"])
    frames = np.array(field["failure_window_raw"])
    tilt = np.array([_tilt_deg(fit_wall_plane(f - offset)[0]) for f in frames])
    jump = np.abs(np.diff(tilt))
    assert np.percentile(jump, 95) < 1.2
    assert jump.mean() < 0.6
