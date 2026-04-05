import py_trees

from task_planner_fsm.bt.adapter import BtStateAdapter
from task_planner_fsm.state import State


class TransitioningState(State):
    def __init__(self, name, transition):
        super().__init__(name)
        self.transition = transition

    def run(self, ctx):
        ctx["ran"] = True

    def check_transition(self, ctx):
        return self.transition


def test_bt_adapter_success_transition():
    ctx = {}
    state = TransitioningState("X", "Done")
    adapter = BtStateAdapter("X", state=state, ctx=ctx, success_transition="Done")
    adapter.initialise()
    status = adapter.update()
    assert status == py_trees.common.Status.SUCCESS


def test_bt_adapter_error_transition():
    ctx = {}
    state = TransitioningState("X", "Error")
    adapter = BtStateAdapter("X", state=state, ctx=ctx, success_transition="Done")
    adapter.initialise()
    status = adapter.update()
    assert status == py_trees.common.Status.FAILURE
