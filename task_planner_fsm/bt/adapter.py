from typing import Optional

import py_trees


class BtStateAdapter(py_trees.behaviour.Behaviour):
    def __init__(self, name, state, ctx, success_transition: Optional[str] = None):
        super().__init__(name=name)
        self._state = state
        self._ctx = ctx
        self._success_transition = success_transition
        self._entered = False

    def initialise(self):
        # py_trees uses British spelling for lifecycle hooks; keep API name as-is.
        if not self._entered:
            self._state.on_enter(self._ctx)
            self._entered = True

    def update(self):
        self._state.run(self._ctx)
        transition = self._state.check_transition(self._ctx)
        if transition == "Error" or self._ctx.get("error_triggered"):
            return py_trees.common.Status.FAILURE
        if self._success_transition and transition == self._success_transition:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Map py_trees termination to legacy state cleanup.

        Called whenever the behavior leaves RUNNING state (success, failure, or interruption).
        It ensures the wrapped FSM state's on_exit hook is invoked exactly once per entry.
        """
        if self._entered:
            self._state.on_exit(self._ctx)
            self._entered = False
