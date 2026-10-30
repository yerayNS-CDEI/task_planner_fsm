"""Has rtabmap placed the robot in the loaded map yet?

In localization mode rtabmap starts with the robot at the map origin
(``RGBD/StartAtOrigin``) and only moves it to where it really is once a frame
matches the map. Until then map->odom is a guess, and every goal in the map
frame (a coverage waypoint, a wall standoff, a sweep line) drives the base
somewhere else. rtabmap reports each match on ``/rtabmap/localization_pose``
with its covariance; that message is the "the robot is localized" signal.

Two rtabmap habits decide how it has to be read (rtabmap_slam CoreWrapper and
corelib Rtabmap::process):

- the pose is published only while somebody subscribes, and only on an update
  that matched the map;
- once two matches sit in its odometry cache, a robot standing still is not
  matched again, so nothing more is published.

So a subscriber created after rtabmap first localized can wait for ever for a
robot that does not move. ObjectID's gate did exactly that, 4 of 11 times. The
fix is to subscribe once, in fsm_node, before any stack is launched, and keep
the last fix in ctx (``record_fix``). Whoever launches a new rtabmap forgets it
first (``forget_fix``): the new one has not localized yet.

ctx knobs (all optional):
  localization_check      false: never wait (default true)
  localization_topic      default /rtabmap/localization_pose (read at node start)
  localization_max_xy_std max horizontal std-dev, sqrt(cov_xx+cov_yy), m (0.25)
  localization_max_yaw_std  max yaw std-dev, rad (0.20)
  localization_timeout    seconds to wait before giving up (60)
  localization_required   true: giving up is an error; false: warn and go (true)
"""

import math
import time

FIX_KEY = "localization_fix"
DEFAULT_TOPIC = "/rtabmap/localization_pose"


def record_fix(ctx, msg):
    """Keep a ``PoseWithCovarianceStamped`` from rtabmap as the latest fix."""
    cov = msg.pose.covariance   # 6x6 row-major: xx=0, yy=7, yaw=35
    p = msg.pose.pose.position
    ctx[FIX_KEY] = {
        "x": float(p.x),
        "y": float(p.y),
        "xy_std": math.sqrt(max(float(cov[0]) + float(cov[7]), 0.0)),
        "yaw_std": math.sqrt(max(float(cov[35]), 0.0)),
        "stamp": time.time(),
    }


def forget_fix(ctx):
    """A new rtabmap is about to start: whatever the old one matched is void."""
    ctx.pop(FIX_KEY, None)


def check_enabled(ctx) -> bool:
    return bool(ctx.get("localization_check", True))


def _limits(ctx):
    return (float(ctx.get("localization_max_xy_std", 0.25)),
            float(ctx.get("localization_max_yaw_std", 0.20)))


def is_localized(ctx) -> bool:
    """A fix is in and it is good enough to drive on."""
    fix = ctx.get(FIX_KEY)
    if not fix:
        return False
    max_xy, max_yaw = _limits(ctx)
    return fix["xy_std"] <= max_xy and fix["yaw_std"] <= max_yaw


class LocalizationWait:
    """Non-blocking wait for a good fix, polled from a state's tick.

    ``start()`` when the wait begins, then ``poll(ctx)`` every tick:
    ``"ok"`` to go, ``"pending"`` to keep the robot still, ``"bad"`` when the
    timeout ran out and ``localization_required`` makes that an error.
    """

    LOG_EVERY_S = 5.0

    def __init__(self, owner: str, before: str):
        self.owner = owner      # state name, for the log prefix
        self.before = before    # what is held back, e.g. "planning the coverage route"
        self.started = None
        self._last_log = None

    def start(self):
        self.started = time.monotonic()
        self._last_log = None

    def waited_s(self) -> float:
        return 0.0 if self.started is None else time.monotonic() - self.started

    def poll(self, ctx) -> str:
        log = ctx["node"].get_logger()
        if not check_enabled(ctx):
            return "ok"
        if self.started is None:
            self.start()

        fix = ctx.get(FIX_KEY)
        max_xy, max_yaw = _limits(ctx)
        if is_localized(ctx):
            log.info(
                f"[{self.owner}] Localized in the map at ({fix['x']:.2f}, {fix['y']:.2f}): "
                f"xy_std={fix['xy_std']:.3f} m (<= {max_xy:.3f}), "
                f"yaw_std={fix['yaw_std']:.3f} rad (<= {max_yaw:.3f})."
            )
            return "ok"

        waited = self.waited_s()
        timeout = float(ctx.get("localization_timeout", 60.0))
        if waited < timeout:
            now = time.monotonic()
            if self._last_log is None or now - self._last_log >= self.LOG_EVERY_S:
                self._last_log = now
                topic = ctx.get("localization_topic", DEFAULT_TOPIC)
                if fix:
                    log.warn(
                        f"[{self.owner}] Not {self.before} yet: last fix on {topic} is too "
                        f"loose (xy_std={fix['xy_std']:.3f}/{max_xy:.3f} m, "
                        f"yaw_std={fix['yaw_std']:.3f}/{max_yaw:.3f} rad; "
                        f"{waited:.0f}/{timeout:.0f}s)."
                    )
                else:
                    log.warn(
                        f"[{self.owner}] Not {self.before} yet: no rtabmap fix on {topic} "
                        f"({waited:.0f}/{timeout:.0f}s). The robot stays still; if its "
                        f"sensors see nothing the map knows, turn it with the joystick."
                    )
            return "pending"

        if bool(ctx.get("localization_required", True)):
            log.error(
                f"[{self.owner}] Robot not localized in the map after {timeout:.0f}s; "
                f"not {self.before}."
            )
            return "bad"
        log.warn(
            f"[{self.owner}] Robot not localized after {timeout:.0f}s, but "
            f"localization_required=false: {self.before} anyway."
        )
        return "ok"
