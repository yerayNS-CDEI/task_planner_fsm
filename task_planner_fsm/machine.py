import time
import traceback
from task_planner_fsm.states.proc_utils import install_global_cleanup

class StateMachine:
    def __init__(self, states, initial_state, ctx):
        self.states = {s.name: s for s in states}
        self.ctx = ctx
        self.current_state = self.states[initial_state]
        self.ctx["fsm_initial_state"] = initial_state
        self.ctx["is_initial_entry"] = True
        self.ctx["last_state"] = None   # initialization
        self._retried_current_state = False
        self._state_enter_time_monotonic = time.monotonic()
        self.ctx.setdefault('mapping_cmd', ['ros2', 'launch', 'navi_wall', 'global_exploration.launch.py', 'headless:=true'])
        install_global_cleanup(self.ctx)
        self.current_state.on_enter(self.ctx)
        self.ctx["is_initial_entry"] = False
        self._set_state_status(
            self.current_state.name,
            phase="entered",
            summary=f"Entered {self.current_state.name}",
        )
        self._publish_current_state(self.current_state.name)
        self._publish_state_status(self.current_state.name)
        self._publish_event(
            "entered",
            state_name=self.current_state.name,
            summary=f"Entered {self.current_state.name}",
        )

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

    def _set_state_status(
        self,
        state_name: str,
        *,
        phase: str = "running",
        summary: str = "",
        data=None,
        progress_current=None,
        progress_total=None,
        level: str = "info",
    ):
        set_status = self.ctx.get("set_fsm_status")
        if callable(set_status):
            set_status(
                state_name,
                phase=phase,
                summary=summary,
                data=data,
                progress_current=progress_current,
                progress_total=progress_total,
                level=level,
            )

    def _publish_state_status(self, state_name: str):
        snapshot = dict(self.ctx.get("_fsm_status", {}))
        snapshot["state"] = state_name
        snapshot["elapsed_s"] = round(
            max(0.0, time.monotonic() - self._state_enter_time_monotonic),
            3,
        )
        snapshot.setdefault("phase", "running")
        snapshot.setdefault("summary", f"Running {state_name}")
        snapshot.setdefault("level", "info")

        publish_status = self.ctx.get("publish_fsm_status")
        if callable(publish_status):
            try:
                publish_status(snapshot)
            except Exception as exc:
                self.ctx["node"].get_logger().warn(f"[FSM] Failed to publish status: {exc}")

    def _publish_event(self, event_type: str, *, state_name: str, summary: str, details=None, level: str = "info"):
        publish_event = self.ctx.get("publish_fsm_event")
        if callable(publish_event):
            try:
                publish_event(
                    event_type,
                    state_name=state_name,
                    summary=summary,
                    details=details,
                    level=level,
                )
            except Exception as exc:
                self.ctx["node"].get_logger().warn(f"[FSM] Failed to publish event: {exc}")

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
            self._publish_event(
                "retry",
                state_name=state_name,
                summary=f"Retrying {state_name} once",
                details={"reason": reason},
                level="warn",
            )
            return state_name, f"retry_once:{reason}"

        node.get_logger().error(
            f"[FSM] {reason} in '{state_name}' after retry. Transitioning to 'Error'."
        )
        self._publish_event(
            "error",
            state_name=state_name,
            summary=f"{state_name} failed after retry",
            details={"reason": reason},
            level="error",
        )
        self._retried_current_state = False
        return "Error", f"to_error_after_retry:{reason}"

    def _apply_transition(self, next_state: str, reason: str = ""):
        previous_state = self.current_state.name
        self._set_state_status(
            previous_state,
            phase="transitioning",
            summary=f"Transitioning to {next_state}",
            data={"to": next_state, "reason": reason},
        )
        self._publish_state_status(previous_state)
        self.current_state.on_exit(self.ctx)
        self.ctx["last_state"] = previous_state
        self._publish_transition(previous_state, next_state, reason)
        self._publish_event(
            "transition",
            state_name=previous_state,
            summary=f"{previous_state} -> {next_state}",
            details={"to": next_state, "reason": reason},
        )
        self.current_state = self.states[next_state]
        if next_state != previous_state:
            self._retried_current_state = False
        self.ctx["is_initial_entry"] = False
        self._state_enter_time_monotonic = time.monotonic()
        self.current_state.on_enter(self.ctx)
        self._set_state_status(
            self.current_state.name,
            phase="entered",
            summary=f"Entered {self.current_state.name}",
            data={"from": previous_state, "reason": reason},
        )
        self._publish_current_state(self.current_state.name)
        self._publish_state_status(self.current_state.name)
        self._publish_event(
            "entered",
            state_name=self.current_state.name,
            summary=f"Entered {self.current_state.name}",
            details={"from": previous_state, "reason": reason},
        )

    def step(self):
        node = self.ctx["node"]
        try:
            # Republish current state every cycle so late subscribers (CLI / RViz) always see it.
            self._publish_current_state(self.current_state.name)
            self.current_state.run(self.ctx)
            current_snapshot = self.ctx.setdefault("_fsm_status", {})
            current_snapshot.setdefault("state", self.current_state.name)
            current_snapshot.setdefault("summary", f"Running {self.current_state.name}")
            current_snapshot.setdefault("level", "info")
            if current_snapshot.get("phase") in (None, "entered"):
                current_snapshot["phase"] = "running"
            self._publish_state_status(self.current_state.name)
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
            self._publish_event(
                "exception",
                state_name=self.current_state.name,
                summary=f"Unhandled exception in {self.current_state.name}",
                details={"traceback": traceback.format_exc()},
                level="error",
            )
            self.ctx["error_triggered"] = True
            next_state, transition_reason = self._resolve_error_transition("Unhandled exception")
            if next_state and next_state in self.states:
                self._apply_transition(next_state, transition_reason)
