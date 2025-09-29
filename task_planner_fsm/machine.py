from task_planner_fsm.states.proc_utils import install_global_cleanup

class StateMachine:
    def __init__(self, states, initial_state, ctx):
        self.states = {s.name: s for s in states}
        self.ctx = ctx
        self.current_state = self.states[initial_state]
        self.ctx["last_state"] = None   # initialization
        self.ctx.setdefault('mapping_cmd', ['ros2', 'launch', 'navi_wall', 'global_exploration.launch.py'])
        install_global_cleanup(self.ctx)
        self.current_state.on_enter(self.ctx)

    def step(self):
        self.current_state.run(self.ctx)
        next_state = self.current_state.check_transition(self.ctx)

        node = self.ctx["node"]
        # node.get_logger().info(f"[FSM] Current state: {self.current_state.name}, next state: {next_state}")

        if next_state and next_state in self.states:
            # self.get_logger().info(f"Transitioning from {self.current_state.name} to {next_state}")
            self.current_state.on_exit(self.ctx)
            self.ctx["last_state"] = self.current_state.name
            self.current_state = self.states[next_state]
            self.current_state.on_enter(self.ctx)
