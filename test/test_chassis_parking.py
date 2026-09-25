"""Chassis parking (utils/chassis_parking.py) and its use at the end of HomePosition.

HomePosition parks once the robot is home, so the next run -- a /fsm/restart
included -- starts with the chassis in line with the turret, like a fresh one.
sim_controller follows commands poorly once that angle passes ~90 deg (the
chassis has to drive in reverse and its pull compensation takes over).

Run with:

    python3 -m pytest test/test_chassis_parking.py -v
"""

import types

from task_planner_fsm.states.home_position import HomePosition
from task_planner_fsm.utils.chassis_parking import ChassisParker


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kw):
        self.lines.append(("info", msg))

    def warn(self, msg, **kw):
        self.lines.append(("warn", msg))

    def error(self, msg, **kw):
        self.lines.append(("error", msg))


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _Client:
    def __init__(self, node, name, available):
        self.node, self.name, self.available = node, name, available

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, req):
        self.node.calls.append((self.name, req))
        if self.name.endswith("set_parameters"):
            ok = types.SimpleNamespace(successful=True, reason="")
            return _Future(types.SimpleNamespace(results=[ok]))
        return _Future(types.SimpleNamespace(success=True, message="parking"))


class _Node:
    def __init__(self, available=("/sim_controller/set_parameters", "/sim_controller/park_now")):
        self.logger = _Logger()
        self.available = set(available)
        self.calls = []

    def get_logger(self):
        return self.logger

    def create_client(self, srv_type, name):
        return _Client(self, name, name in self.available)


def enabled_values(node):
    return [req.parameters[0].value.bool_value
            for name, req in node.calls if name.endswith("set_parameters")]


def drive(parker, ctx, active_sequence, max_ticks=20):
    """Tick the parker, feeding parking_active from active_sequence (last value sticks)."""
    for i in range(max_ticks):
        ctx["parking_active"] = active_sequence[min(i, len(active_sequence) - 1)]
        if parker.step(ctx):
            return i + 1
    return None


def test_full_maneuver_enables_parks_waits_and_disables():
    node = _Node()
    ctx = {"node": node}
    parker = ChassisParker("HomePosition", "home_position")
    # parking_active: not yet, then active for two ticks, then done.
    assert drive(parker, ctx, [False, False, False, False, True, True, False]) is not None
    assert enabled_values(node) == [True, False]
    assert [n for n, _ in node.calls].count("/sim_controller/park_now") == 1
    assert any("Chassis aligned" in text for _, text in node.logger.lines)


def test_already_aligned_finishes_after_the_grace():
    node = _Node()
    ctx = {"node": node, "home_position_park_grace_s": 0.0}
    parker = ChassisParker("HomePosition", "home_position")
    assert drive(parker, ctx, [False]) is not None
    assert enabled_values(node) == [True, False]
    assert any("already aligned" in text for _, text in node.logger.lines)


def test_opt_out_knob_is_per_owner():
    node = _Node()
    ctx = {"node": node, "home_position_park_base": False}
    assert ChassisParker("HomePosition", "home_position").step(ctx) is True
    assert node.calls == []
    # ScanWall's knob is separate: it still parks.
    assert ChassisParker("ScanWall", "scan_wall").step(ctx) is False
    assert node.calls


def test_missing_park_service_reverts_the_parameter():
    node = _Node(available=("/sim_controller/set_parameters",))
    ctx = {"node": node}
    parker = ChassisParker("HomePosition", "home_position")
    assert drive(parker, ctx, [False]) is not None
    assert enabled_values(node) == [True, False]
    assert not any(n == "/sim_controller/park_now" for n, _ in node.calls)


def test_missing_parameter_service_skips_without_parking():
    node = _Node(available=())
    ctx = {"node": node}
    assert ChassisParker("HomePosition", "home_position").step(ctx) is True
    assert node.calls == []


def test_reset_starts_a_new_maneuver():
    node = _Node()
    ctx = {"node": node, "home_position_park_grace_s": 0.0}
    parker = ChassisParker("HomePosition", "home_position")
    drive(parker, ctx, [False])
    parker.reset()
    assert parker.done is False
    drive(parker, ctx, [False])
    assert enabled_values(node) == [True, False, True, False]


# ---------------------------------------------------------------------------
# HomePosition
# ---------------------------------------------------------------------------

def test_home_position_parks_before_finishing():
    node = _Node()
    ctx = {"node": node, "nav_client": object(), "home_position_park_grace_s": 0.0}
    home = HomePosition("HomePosition")
    home.on_enter(ctx)

    # Still driving home: no parking, no transition.
    home.goal_sent = True
    assert home.check_transition(ctx) is None
    assert node.calls == []

    home.navigation_done = True          # what result_callback sets
    assert home.check_transition(ctx) is None, "must not finish before parking"
    for _ in range(20):
        home.run(ctx)
        if home.check_transition(ctx):
            break
    assert home.check_transition(ctx) == "Finished"
    assert enabled_values(node) == [True, False]


def test_home_position_re_entry_parks_again():
    node = _Node()
    ctx = {"node": node, "nav_client": object(), "home_position_park_grace_s": 0.0}
    home = HomePosition("HomePosition")
    for _ in range(2):
        home.on_enter(ctx)
        home.navigation_done = True
        for _ in range(20):
            home.run(ctx)
            if home.check_transition(ctx):
                break
        assert home.check_transition(ctx) == "Finished"
    assert enabled_values(node) == [True, False, True, False]
