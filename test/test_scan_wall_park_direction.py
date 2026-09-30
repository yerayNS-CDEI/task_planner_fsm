"""Which way round ScanWall parks the chassis before a sweep.

The base's turret sits d1 ahead of the chassis axle, so the chassis is stable
only while it PUSHES the turret. A serpentine scan runs every second line the
other way along the wall; parked square, that line PULLS the chassis — it
flipped 180 deg mid-sweep on 2026-09-25 and 09-28, and with sim_controller's
pull compensation switched in it mirrored the whole-body controller's sideways
corrections instead (09-28 16:53, line failed on the torque limit). So a line
that runs turret-backward is parked with the chassis turned round (pi).
"""

import math
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("arm_control.srv")           # the FSM state imports these
pytest.importorskip("ur_msgs.srv")

from task_planner_fsm.states.scan_wall import ScanWall, park_target_phi   # noqa: E402

# The wall of the 2026-09-28 runs, and the turret heading the base held on it
# (odom->turret_footprint ~209 deg, map->odom ~identity).
LINE_1 = ((4.71, -2.33), (-1.00, -5.16))     # swept turret-forward (vx +45 mm/s)
LINE_2 = (LINE_1[1], LINE_1[0])              # the serpentine's return (vx -42)
TURRET_YAW = math.radians(209.0)


def test_a_turret_forward_line_parks_square():
    assert park_target_phi(*LINE_1, TURRET_YAW) == 0.0


def test_a_turret_backward_line_parks_turned_round():
    assert park_target_phi(*LINE_2, TURRET_YAW) == pytest.approx(math.pi)


def test_only_the_direction_along_the_turret_matters():
    # 10 deg of heading error either way does not change the answer.
    for off in (-10.0, 10.0):
        yaw = TURRET_YAW + math.radians(off)
        assert park_target_phi(*LINE_1, yaw) == 0.0
        assert park_target_phi(*LINE_2, yaw) == pytest.approx(math.pi)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = rclpy.create_node("scan_wall_park_direction_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def state():
    s = ScanWall("ScanWall")
    s.current_line_z = 1.5
    return s


def test_the_target_follows_the_segment_about_to_be_swept(node, state):
    ctx = {"node": node, "sim": False}
    state._base_xy_yaw_map = lambda ctx: (0.0, 0.0, TURRET_YAW)
    state._segments, state._seg_idx = [LINE_1, LINE_2], 0
    assert state._park_target_for_segment(ctx) == 0.0
    state._seg_idx = 1
    assert state._park_target_for_segment(ctx) == pytest.approx(math.pi)
    # Opt-out: park square as before.
    ctx["scan_wall_park_reverse"] = False
    assert state._park_target_for_segment(ctx) == 0.0


def test_no_turret_pose_parks_square(node, state):
    state._base_xy_yaw_map = lambda ctx: None
    state._segments, state._seg_idx = [LINE_2], 0
    assert state._park_target_for_segment({"node": node}) == 0.0


class _Client:
    def __init__(self):
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, req):
        self.requests.append(req)
        return "future"


def test_the_target_travels_with_the_enable_and_is_reset_with_the_disable(node, state):
    state.park_enable_client = _Client()
    ctx = {"node": node}
    state._send_park_enabled(ctx, True, math.pi)
    state._send_park_enabled(ctx, False, 0.0)
    enable, disable = state.park_enable_client.requests
    # enable_park_service FIRST, so an old sim_controller rejects only the target.
    assert [p.name for p in enable.parameters] == ["enable_park_service", "park_target_phi"]
    assert enable.parameters[0].value.bool_value is True
    assert enable.parameters[1].value.double_value == pytest.approx(math.pi)
    assert disable.parameters[0].value.bool_value is False
    assert disable.parameters[1].value.double_value == 0.0


def _future(*ok):
    results = [SimpleNamespace(successful=o, reason="" if o else "parameter not declared")
               for o in ok]
    return SimpleNamespace(result=lambda: SimpleNamespace(results=results))


def test_an_old_sim_controller_still_parks_square_rather_than_skipping(node, state):
    """A controller built before park_target_phi rejects it; the enable must
    still count, and the state must know the park now goes to 0."""
    ctx = {"node": node}
    future = _future(True, False)
    state._park_target = math.pi
    state._check_park_target_set(ctx, future)
    assert state._park_target == 0.0
    assert state._param_set_ok(ctx, future, "enable", only_first=True)
    assert not state._param_set_ok(ctx, future, "enable")
