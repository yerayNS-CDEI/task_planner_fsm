"""Unit tests for costmap_utils.nav_bt_xml (the BT sent with every Nav2 goal).

With the Jetson/PC2 split, bt_navigator runs on the other computer and opens
the goal's behavior_tree path there, so the path must come from bt_navigator's
own default tree, never from this computer's install. No ROS graph is used: the
``ros2 param get`` call is replaced by a fake.

Run with:

    python3 -m pytest test/test_nav_bt_xml.py -v
"""

import subprocess

import pytest

from task_planner_fsm.utils import costmap_utils

PC2_TREES = "/home/pc2/ros2_ws/install/navi_wall/share/navi_wall/config/behavior_trees"


class _FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, msg):
        pass

    def warn(self, msg):
        self.warnings.append(msg)


class _FakeNode:
    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


@pytest.fixture
def param_get(monkeypatch):
    """Make ``ros2 param get`` print ``stdout``; returns the list of calls."""
    calls = []

    def install(stdout="", raises=None):
        def fake_run(cmd, **_kwargs):
            calls.append(cmd)
            if raises is not None:
                raise raises
            return subprocess.CompletedProcess(cmd, 0, stdout=stdout, stderr="")
        monkeypatch.setattr(costmap_utils.subprocess, "run", fake_run)
        return calls
    return install


def test_the_tree_is_found_next_to_bt_navigators_default(param_get):
    calls = param_get(f"String value is: {PC2_TREES}/navigate_to_pose_w_backout_recovery.xml\n")
    ctx = {"node": _FakeNode()}
    assert costmap_utils.nav_bt_xml(ctx, "Test") == f"{PC2_TREES}/navigate_to_pose_no_backout.xml"
    assert calls[0][:3] == ["ros2", "param", "get"]
    assert calls[0][3:] == ["/bt_navigator", "default_nav_to_pose_bt_xml"]


def test_the_answer_is_cached_for_the_session(param_get):
    calls = param_get(f"String value is: {PC2_TREES}/navigate_to_pose_w_backout_recovery.xml\n")
    ctx = {"node": _FakeNode()}
    first = costmap_utils.nav_bt_xml(ctx, "Test")
    assert costmap_utils.nav_bt_xml(ctx, "Test") == first
    assert len(calls) == 1


def test_a_stock_default_tree_falls_back_to_the_default(param_get):
    param_get("String value is: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/"
              "navigate_to_pose_w_replanning_and_recovery.xml\n")
    ctx = {"node": _FakeNode()}
    assert costmap_utils.nav_bt_xml(ctx, "Test") == ""
    assert ctx["node"].logger.warnings


@pytest.mark.parametrize("kwargs", [{"stdout": ""}, {"raises": subprocess.TimeoutExpired("ros2", 10)}])
def test_no_answer_falls_back_and_asks_again_next_time(param_get, kwargs):
    calls = param_get(**kwargs)
    ctx = {"node": _FakeNode()}
    assert costmap_utils.nav_bt_xml(ctx, "Test") == ""
    assert costmap_utils.nav_bt_xml(ctx, "Test") == ""
    assert len(calls) == 2


def test_the_nav_bt_xml_parameter_overrides_everything(param_get):
    calls = param_get()
    ctx = {"node": _FakeNode(), "nav_bt_xml": "/custom/tree.xml"}
    assert costmap_utils.nav_bt_xml(ctx, "Test") == "/custom/tree.xml"
    assert calls == []
