import traceback
from task_planner_fsm.states.proc_utils import install_global_cleanup
from task_planner_fsm.context import FsmContext

class StateMachine:
    def __init__(self, states, initial_state, ctx: FsmContext):
        self.states = {s.name: s for s in states}
        self.ctx = ctx
        self.current_state = self.states[initial_state]
        self.ctx["fsm_initial_state"] = initial_state
        self.ctx["is_initial_entry"] = True
        self.ctx["last_state"] = None   # initialization
        self._retried_current_state = False
        self.ctx.setdefault('mapping_cmd', ['ros2', 'launch', 'navi_wall', 'global_exploration.launch.py', 'headless:=true'])
        install_global_cleanup(self.ctx)
        self.current_state.on_enter(self.ctx)
        self.ctx["is_initial_entry"] = False
        self._publish_current_state(self.current_state.name)

    def _publish_current_state(self, state_name: str):
        publish_current = self.ctx.get("publish_fsm_current")
        if callable(publish_current):
            try:
                publish_current(state_name)
            except Exception as exc:
                self.ctx["node"].get_logger().warn(f"[FSM] Failed to publish current state: {exc}")

    def _publish_transition(self, from_state: str, to_state: str, reason: str = ""):
        publish_transition = self.ctx.get("publish_fsm_transition")
        if callable(publish_transition):
            try:
                publish_transition(from_state, to_state, reason)
            except Exception as exc:
                self.ctx["node"].get_logger().warn(f"[FSM] Failed to publish transition: {exc}")

    def _resolve_error_transition(self, reason: str):
        node = self.ctx["node"]
        state_name = self.current_state.name

        if state_name == "Error" or "Error" not in self.states:
            return "Error", "error_fallback"

        if not self._retried_current_state:
            self._retried_current_state = True
            self.ctx["error_triggered"] = False
            node.get_logger().warn(
                f"[FSM] {reason} in '{state_name}'. Retrying this state once before 'Error'."
            )
            return state_name, f"retry_once:{reason}"

        node.get_logger().error(
            f"[FSM] {reason} in '{state_name}' after retry. Transitioning to 'Error'."
        )
        self._retried_current_state = False
        return "Error", f"to_error_after_retry:{reason}"

    def _apply_transition(self, next_state: str, reason: str = ""):
        previous_state = self.current_state.name
        self.current_state.on_exit(self.ctx)
        self.ctx["last_state"] = previous_state
        self._publish_transition(previous_state, next_state, reason)
        self.current_state = self.states[next_state]
        if next_state != previous_state:
            self._retried_current_state = False
        self.ctx["is_initial_entry"] = False
        self.current_state.on_enter(self.ctx)
        self._publish_current_state(self.current_state.name)

    def step(self):
        node = self.ctx["node"]
        try:
            # Republish current state every cycle so late subscribers (CLI / RViz) always see it.
            self._publish_current_state(self.current_state.name)
            self.current_state.run(self.ctx)
            next_state = self.current_state.check_transition(self.ctx)
            transition_reason = "state_transition"

            if next_state == "Error":
                next_state, transition_reason = self._resolve_error_transition("Error requested")

            if next_state and next_state in self.states:
                self._apply_transition(next_state, transition_reason)
        except Exception:
            node.get_logger().error(
                f"[FSM] Unhandled exception in state '{self.current_state.name}':\n"
                + traceback.format_exc()
            )
            self.ctx["error_triggered"] = True
            next_state, transition_reason = self._resolve_error_transition("Unhandled exception")
            if next_state and next_state in self.states:
                self._apply_transition(next_state, transition_reason)
