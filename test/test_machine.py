from task_planner_fsm.machine import StateMachine
from task_planner_fsm.state import State


class DummyNode:
    class _Logger:
        def info(self, *_):
            pass

        def warn(self, *_):
            pass

        def error(self, *_):
            pass

    def get_logger(self):
        return self._Logger()


class StateA(State):
    def on_enter(self, ctx):
        ctx["entered_a"] = True

    def run(self, ctx):
        ctx["ran_a"] = True

    def check_transition(self, ctx):
        return "StateB"


class StateB(State):
    def on_enter(self, ctx):
        ctx["entered_b"] = True


def test_machine_transitions_to_next_state():
    ctx = {"node": DummyNode()}
    machine = StateMachine(
        states=[StateA("StateA"), StateB("StateB")],
        initial_state="StateA",
        ctx=ctx,
    )
    machine.step()
    assert ctx["ran_a"] is True
    assert ctx["entered_b"] is True
