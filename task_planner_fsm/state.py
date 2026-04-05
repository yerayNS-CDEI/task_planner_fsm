from task_planner_fsm.context import FsmContext


class State:
    def __init__(self, name):
        self.name = name

    def on_enter(self, ctx: FsmContext):
        pass

    def run(self, ctx: FsmContext):
        pass

    def check_transition(self, ctx: FsmContext):
        return None

    def on_exit(self, ctx: FsmContext):
        pass
