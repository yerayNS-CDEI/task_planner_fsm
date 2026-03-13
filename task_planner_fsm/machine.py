import traceback
from task_planner_fsm.states.proc_utils import install_global_cleanup

class StateMachine:
    def __init__(self, states, initial_state, ctx):
        self.states = {s.name: s for s in states}
        self.ctx = ctx
        self.current_state = self.states[initial_state]
        self.ctx["last_state"] = None   # initialization
        self._retried_current_state = False
        self.ctx.setdefault('mapping_cmd', ['ros2', 'launch', 'navi_wall', 'global_exploration.launch.py'])
        install_global_cleanup(self.ctx)
        self.current_state.on_enter(self.ctx)

    def _resolve_error_transition(self, reason: str):
        node = self.ctx["node"]
        state_name = self.current_state.name

        if state_name == "Error" or "Error" not in self.states:
            return "Error"

        if not self._retried_current_state:
            self._retried_current_state = True
            self.ctx["error_triggered"] = False
            node.get_logger().warn(
                f"[FSM] {reason} in '{state_name}'. Retrying this state once before 'Error'."
            )
            return state_name

        node.get_logger().error(
            f"[FSM] {reason} in '{state_name}' after retry. Transitioning to 'Error'."
        )
        self._retried_current_state = False
        return "Error"

    def _apply_transition(self, next_state: str):
        previous_state = self.current_state.name
        self.current_state.on_exit(self.ctx)
        self.ctx["last_state"] = previous_state
        self.current_state = self.states[next_state]
        if next_state != previous_state:
            self._retried_current_state = False
        self.current_state.on_enter(self.ctx)

    def step(self):
        node = self.ctx["node"]
        try:
            self.current_state.run(self.ctx)
            next_state = self.current_state.check_transition(self.ctx)

            if next_state == "Error":
                next_state = self._resolve_error_transition("Error requested")

            if next_state and next_state in self.states:
                self._apply_transition(next_state)
        except Exception:
            node.get_logger().error(
                f"[FSM] Unhandled exception in state '{self.current_state.name}':\n"
                + traceback.format_exc()
            )
            self.ctx["error_triggered"] = True
            next_state = self._resolve_error_transition("Unhandled exception")
            if next_state and next_state in self.states:
                self._apply_transition(next_state)
