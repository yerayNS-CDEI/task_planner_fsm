"""ScanWall's side of the whole-body sweep: the gate and the status mapping.

Both are easy to break silently — a mis-mapped status would leave ``sweep_wait``
waiting forever, and a gate that opens on the real robot would hand the arm to a
streaming velocity controller that cannot coexist with ``force_mode``.
"""

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("arm_control.srv")           # the FSM state imports these
pytest.importorskip("ur_msgs.srv")

from action_msgs.msg import GoalStatus            # noqa: E402
from std_msgs.msg import String                   # noqa: E402

from task_planner_fsm.states.scan_wall import ScanWall   # noqa: E402


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = rclpy.create_node("scan_wall_wbc_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def state():
    return ScanWall("ScanWall")


def test_the_sweep_is_off_unless_it_is_asked_for(state, node):
    assert not state._wbc_enabled({"node": node, "sim": True})


def test_the_sweep_runs_in_simulation_when_asked(state, node):
    assert state._wbc_enabled({"node": node, "sim": True, "sweep_use_wbc": True})


def test_asking_for_the_sweep_on_hardware_gets_the_sweep(state, node):
    """One flag decides. There is no second opt-in for the real robot.

    There used to be (`wbc_allow_real`), on the grounds that giving up the
    force_mode press was a separate decision. It is not: the UR driver refuses
    force mode alongside a streaming controller outright, so asking for the
    whole-body sweep IS asking to go without the press. And the old shape failed
    badly -- forgetting the second flag fell back to the Nav2 sweep with only a
    warning, so a run could look like a whole-body test without being one.
    """
    assert state._wbc_enabled({"node": node, "sim": False, "sweep_use_wbc": True})


def test_force_mode_is_not_attempted_when_the_sweep_is_whole_body(state, node):
    """The driver would refuse it, and asking cost a full contact-wait timeout.

    The service stays advertised while the controller is inactive, so the call
    was accepted, `force_mode_active` was set, and `_wall_contact_ready` then
    waited out its timeout for a contact that could never arrive.
    """
    ctx = {"node": node, "sim": False, "sweep_use_wbc": True}
    state._start_force_mode(ctx)
    assert state.force_mode_active is False
    assert state.force_mode_start_client is None       # never even built a client
    # ... and with the whole-body sweep off, the press is still attempted.
    assert state._start_force_mode({"node": node, "sim": True}) is None


def test_node_status_maps_onto_the_goal_status_sweep_wait_reads(state):
    assert state._nav_status is None
    state._on_wbc_status(String(data="running"), {})
    assert state._nav_status is None               # still in flight

    state._on_wbc_status(String(data="succeeded"), {})
    assert state._nav_status == GoalStatus.STATUS_SUCCEEDED


def test_a_failure_is_recorded_with_its_reason(state):
    state._on_wbc_status(String(data="failed: standoff 0.55 m is 0.35 m off the target"), {})
    assert state._nav_status == -1
    assert "standoff" in state._wbc_failure


def test_the_transit_goal_keeps_its_standoff_when_the_arm_is_folded(state, node, monkeypatch):
    """wall_parallel_goal exists because the arm is stretched out during a
    transit. Folding removes that premise — and keeping it strands the base too
    far from the wall for the arm to reach, which lost a segment in simulation."""
    import task_planner_fsm.states.scan_wall as sw

    calls = []
    monkeypatch.setattr(sw, "base_standoff_goal", lambda *a, **k: (4.0, 0.9))
    monkeypatch.setattr(sw, "wall_parallel_goal",
                        lambda *a, **k: calls.append("stripped") or (4.0, 0.7))

    ctx = {"node": node, "sim": True, "base_position": None}
    _, goal = state._transit_goal_for(dict(ctx, scan_wall_fold_for_transit=True),
                                      (4.0, 1.4, 1.5))
    assert goal == (4.0, 0.9) and not calls, "folded transit must keep the standoff"

    _, goal = state._transit_goal_for(dict(ctx, scan_wall_fold_for_transit=False),
                                      (4.0, 1.4, 1.5))
    assert goal == (4.0, 0.7) and calls == ["stripped"], "unfolded transit still slides parallel"


def test_a_dead_sweep_process_ends_the_wait(state, node):
    class Dead:
        returncode = 1

        def poll(self):
            return 1

    ctx = {"node": node, "sim": True, "sweep_use_wbc": True, "wbc_sweep_proc": Dead()}
    state._wbc_sweep_tick(ctx)
    assert state._nav_status == -1


def test_the_watchdog_leaves_a_live_sweep_alone(state, node):
    class Alive:
        def poll(self):
            return None

    ctx = {"node": node, "sim": True, "sweep_use_wbc": True, "wbc_sweep_proc": Alive()}
    state._wbc_started_at = 1e12
    state._wbc_sweep_tick(ctx)
    assert state._nav_status is None
