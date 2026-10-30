"""Localize: keep the robot still until rtabmap has placed it in the map.

A start past ObjectID brings the localization stack up in the bootstrap and
used to go straight into its first state, so NavigateToTarget (or ScanWall's
own transits) drove on map->odom while rtabmap still had the robot at the map
origin. fsm_node now enters this state first and names the real start in
``ctx['localize_next_state']``; ObjectID, the full flow's first motion in the
localized map, has the same gate built in. See utils/localization.py.
"""

from ..state import State
from ..utils.localization import LocalizationWait

NEXT_STATE_KEY = "localize_next_state"


class Localize(State):
    def __init__(self, name):
        super().__init__(name)
        self._wait = LocalizationWait(name, "moving")
        self._done = False

    def on_enter(self, ctx):
        ctx["error_triggered"] = False
        self._done = False
        self._wait.start()
        ctx["node"].get_logger().info(
            f"[{self.name}] Waiting for rtabmap to localize the robot before "
            f"{ctx.get(NEXT_STATE_KEY)} moves it."
        )

    def run(self, ctx):
        self.set_activity(ctx, "Waiting to be localized in the map before moving")
        status = self._wait.poll(ctx)
        if status == "ok":
            self._done = True
        elif status == "bad":
            self.fail(ctx, f"not localized in the map after {self._wait.waited_s():.0f}s")

    def check_transition(self, ctx):
        if self._done:
            return ctx.get(NEXT_STATE_KEY)
        if ctx.get("error_triggered"):
            return "Error"
        return None
