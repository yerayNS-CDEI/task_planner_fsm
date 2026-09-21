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
# Against the robot: the calibration data, replayed
# ----------------------------------------------------------------------
#
# test/fixtures/plate_ranges_2026_09_17.json holds, for 17 poses the arm
# held still in front of a concrete wall, the mean of the six RAW published
# ranges and the wall's normal and distance in the plate frame from the arm's
# forward kinematics; the 104 raw range frames from the approach that ended
# in the 2026-09-15 31 N overload; and, since 2026-09-21, ten raw frames with
# the plate PARALLEL to a flat wall (all four GPR casters touching).
#
# The reader subtracts a constant offset per sensor before publishing
# (arm_control sensors/plate_calibration.py); the fixture carries a copy and
# these tests apply it to the raw data. What each part of the data can
# validate is different, and the tests are careful about it:
#
#   * The parallel pose is a direct measurement: the six true ranges are
#     equal, so corrected ranges must fit FLAT. This is the anchor.
#   * The FK poses validate the RELATIVE geometry only. The 2026-09-17 fit
#     solved for the wall plane's orientation together with the offsets, and
#     the two are not separable — a wrong wall tilt and a diagonal offset
#     pattern are the same thing to a plane fit. It settled 6.2 deg off, and
#     the offsets it produced made every pose read 6.2 deg off the real
#     wall (that is what put a corner into the wall first from 09-17 to
#     09-21). So the FK normals in the fixture are all rotated by that
#     constant, and the test checks angles BETWEEN poses, which the constant
#     drops out of.
#
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


def _reader_plane_ranges(frame):
    """Six raw serial values -> metres from the sensor plane, as the reader does."""
    u = [v / 100.0 for v in frame[:3]]
    s = [v / 1000.0 + 0.083 for v in frame[3:]]
    return np.array(u + s)


def test_a_parallel_plate_reads_flat_through_the_calibration(field):
    """The anchor: with all four casters on the wall the plate IS parallel,
    so the corrected fit must say so — under half a degree, at the datum the
    press is re-datumed to (press_contact_distance 0.140)."""
    offset = np.array(field["range_offset_m"])
    frames = np.array(field["parallel_pose_raw_2026_09_21"]["frames_raw_serial"], dtype=float)
    mean = _reader_plane_ranges(frames.mean(axis=0))
    normal, distance, n_valid = fit_wall_plane(mean - offset)
    assert n_valid == 6
    assert _tilt_deg(normal) < 0.5, f"parallel plate reads {_tilt_deg(normal):.2f} deg tilted"
    assert abs(distance - 0.1395) < 0.004
    # Frame by frame too, so the anchor is not an artefact of averaging the
    # centimetre-quantised ultrasonics.
    per_frame = [_tilt_deg(fit_wall_plane(_reader_plane_ranges(f) - offset)[0]) for f in frames]
    assert max(per_frame) < 1.5


def test_the_old_fk_fit_offsets_tilt_a_parallel_plate(field):
    """The 2026-09-17 offsets read the same parallel plate ~6 deg off — the
    constant the FK fit absorbed into them. Kept so the failure mode stays
    documented in a form that runs: if a future re-fit against the arm ever
    reproduces this, it has made the same mistake."""
    old = np.array(field["range_offset_m_2026_09_17_fk_fit"])
    mean = _reader_plane_ranges(
        np.array(field["parallel_pose_raw_2026_09_21"]["frames_raw_serial"], dtype=float).mean(axis=0))
    assert _tilt_deg(fit_wall_plane(mean - old)[0]) > 5.0


def test_calibrated_ranges_keep_the_relative_geometry_the_arm_measured(field):
    """Across the 17 FK poses the angle between any two fitted normals must
    match the angle between the arm's two normals: the wall constant drops
    out, what is left is whether the offsets distort the plate's rotations.
    Corrected: under 1 deg for every pair. Raw: not."""
    offset = np.array(field["range_offset_m"])
    poses = field["poses"]
    fk = [np.array(p["fk_normal_plate"]) for p in poses]
    fit = [fit_wall_plane(np.array(p["raw_mean"]) - offset)[0] for p in poses]
    raw = [fit_wall_plane(np.array(p["raw_mean"]))[0] for p in poses]
    pairs = [(i, j) for i in range(len(poses)) for j in range(i + 1, len(poses))]
    corrected_err = [abs(_angle_deg(fk[i], fk[j]) - _angle_deg(fit[i], fit[j])) for i, j in pairs]
    raw_err = [abs(_angle_deg(fk[i], fk[j]) - _angle_deg(raw[i], raw[j])) for i, j in pairs]
    assert max(corrected_err) < 1.0, f"worst pair {max(corrected_err):.2f} deg"
    assert np.mean(corrected_err) < 0.3
    assert max(raw_err) > 3.0, "raw ranges keep the geometry — the offsets are doing nothing"
    # And the constant itself, so the number in the docs stays tied to data:
    # every FK normal sits the same ~6.2 deg from its corrected fit.
    const = [_angle_deg(a, b) for a, b in zip(fk, fit)]
    assert 5.5 < np.mean(const) < 7.0 and np.std(const) < 0.5


def test_the_approach_that_overloaded_was_two_degrees_off_not_seven(field):
    """The 2026-09-15 31 N overload, re-read. Through the 09-17 offsets it was
    the '7.7 deg off' approach that motivated a week of alignment work; through
    the parallel-pose offsets it was ~2 deg off, and the raw fit (what the
    controller saw at the time) ~1.7. The overload was not a plate arriving
    seven degrees off; it was the base driving a plate that was nearly square."""
    offset = np.array(field["range_offset_m"])
    old = np.array(field["range_offset_m_2026_09_17_fk_fit"])
    frames = np.array(field["failure_window_raw"])
    corrected = np.array([_tilt_deg(fit_wall_plane(f - offset)[0]) for f in frames])
    through_old = np.array([_tilt_deg(fit_wall_plane(f - old)[0]) for f in frames])
    assert 1.0 < corrected.mean() < 3.5
    assert through_old.mean() > 6.0


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
