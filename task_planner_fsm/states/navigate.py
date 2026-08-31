from ..state import State
from ..utils.costmap_utils import (
    base_standoff_goal,
    await_costmap,
    nav_bt_xml,
    plan_wall_partitions,
    reachable_wall_segments,
    publish_wall_segment_markers,
    set_arm_footprint_enabled,
    wall_facing_yaw,
)
from ..utils.wall_approach import should_approach_partition, should_face_wall
from ..utils.wall_partitioning import sweep_line_order
import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.task import Future
from math import atan2, sin, cos
import math, time

class NavigateToTarget(State):
    def __init__(self, name):
        super().__init__(name)
        self.goal_sent = False
        self.navigation_done = False
        self.future = None
        self.verbose = False
        self.waiting = False
        self._costmap_wait_start = None
        self._nav_result_pending = False
        self._nav_status = None
        self._pose_retries = 0
        self._goal_xy = None
        self._goal_yaw = None
        self._nav_goal_handle = None    # in-flight NavigateToPose handle, for early cancel
        self._nav_cancel_sent = False
        self._target_clamped = False
        self._servo_timer = None
        self._servo_start = 0.0
        # True while the fine-correction servo is commanding yaw only.
        self._servo_rotate_only = False

    def on_enter(self, ctx):
        node = ctx["node"]
        node.get_logger().info(f"[{self.name}] Entering navigation state.")
        # Drive to the wall with the arm-aware (dynamic hull) footprint; it is
        # switched to a base-only circle on arrival (result_callback) so the arm
        # footprint touching the wall can't stall the final approach.
        set_arm_footprint_enabled(ctx, True)
        ctx["navigation_success"] = False
        ctx["error_triggered"] = False
        self.goal_sent = False
        self.navigation_done = False
        self.future = None
        self.waiting = False
        self._costmap_wait_start = None
        self._nav_result_pending = False
        self._nav_status = None
        self._pose_retries = 0
        self._goal_xy = None
        self._goal_yaw = None
        self._nav_goal_handle = None    # in-flight NavigateToPose handle, for early cancel
        self._nav_cancel_sent = False
        self._target_clamped = False
        self._stop_fine_correction(ctx)  # clear any stale servo from a re-entry

        try:
            if not rclpy.ok():
                node.get_logger().warn(f"[{self.name}] ROS no está OK; abortando on_enter.")
                ctx["error_triggered"] = True
                return

            # Action clients
            ctx["nav_client"] = ActionClient(node, NavigateToPose, "/navigate_to_pose")
            if not ctx["nav_client"].wait_for_server(timeout_sec=10.0):
                node.get_logger().error("NavigateToPose action server not available after 10 seconds.")
                ctx["error_triggered"] = True
                return
            
        except Exception as e:
            node.get_logger().error(f"[{self.name}] Exception in on_enter: {e}")
            ctx["error_triggered"] = True
            return

        time.sleep(5)

    def run(self, ctx):
        node = ctx["node"]
        # nav_client: ActionClient = ctx["nav_client"]
        self.set_activity(ctx, "Navigating to a scan point")

        # if self.result_received:
        #     # Check result status
        #     if self.last_status == 4:  # SUCCEEDED
        #         node.get_logger().info(f"[{self.name}] Navigation successful.")
        #         ctx["navigation_success"] = True
        #     else:
        #         node.get_logger().warn(f"[{self.name}] Navigation failed. Status: {self.last_status}")
        #         ctx["error_triggered"] = True
        #     return
        
        if not self.goal_sent:
            wall_data = ctx.get("target_scan_wall")  # Tuple of (start_point, end_point)
            target_point = ctx.get("target_scan_point", None)

            if not wall_data or not target_point:
                node.get_logger().error(f"[{self.name}] Missing wall data or target point in context.")
                self.fail(ctx, "missing wall data or target point")
                return
            
            if len(wall_data) != 2:
                node.get_logger().error(f"[{self.name}] Invalid wall_data: expected 2 points.")
                ctx["error_triggered"] = True
                return

            # Phase-1 base goals sit near the wall; wait for the Nav2 global
            # costmap to actually cover the target before sending, else the
            # planner fails on an out-of-bounds/unknown goal. Non-blocking poll.
            if ctx.get("scan_phase") != 2:
                decision, self._costmap_wait_start = await_costmap(
                    ctx, self.name, self._costmap_wait_start, target_point
                )
                if decision == "waiting":
                    return

                # Reachability-first target: raw wall endpoints often sit in a
                # corner where no base cell exists within arm reach. Clamp the
                # scan start to the nearest reachable sub-segment of the wall so
                # the base always stays close enough for the arm; ScanWall later
                # sweeps those same segments and skips the gaps.
                if not self._target_clamped:
                    target_point = self._clamp_target_to_segments(
                        ctx, wall_data, target_point
                    )
                    if target_point is None:
                        return  # wall entirely unreachable -> error set
                    self._target_clamped = True

            # Arm-driven sweep: approach the scan pose of the FIRST partition
            # ScanWall will sweep, not the start of the wall. The base then
            # arrives already parked for that partition and ScanWall skips its
            # transit -- which matters because by then the arm is unfolded and
            # Nav2 can barely manoeuvre at the scan standoff (ARM_SWEEP_PLAN §9).
            # None falls back to the wall-start goal below.
            partition_goal = None
            if ctx.get("scan_phase") != 2 and should_approach_partition(ctx):
                partition_goal = self._first_partition_pose(ctx, wall_data, target_point)

            if math.dist(target_point[:2], wall_data[0][:2]) <= math.dist(
                target_point[:2], wall_data[1][:2]
            ):
                other_point = wall_data[1]
            else:
                other_point = wall_data[0]
            
            # Orientation. Two conventions, one per sweep architecture:
            #  - base-driven sweep: head ALONG the wall (start -> far end), so the
            #    base can drive down it with the sensor plate on its left flank.
            #  - arm-driven sweep (see should_face_wall): face INTO the wall, so
            #    the parked base is square to it and the arm sweeps symmetrically
            #    about its centreline. ScanWall then parks each partition at this
            #    same heading, so approaching at it here means the turret is
            #    already right when the first partition transit is skipped as a
            #    no-op. See ARM_SWEEP_PLAN §7.A.
            #
            # The arm pose must match: ArmUnfolding picks it via the same
            # helper, so the plate always ends up aimed at the wall.
            yaw = None
            if partition_goal is not None:
                # The partition pose already carries the wall-facing yaw, turret
                # offset included -- taking it wholesale keeps position and
                # heading from being derived by two different code paths.
                yaw = partition_goal[2]
            elif should_face_wall(ctx):
                yaw = wall_facing_yaw(ctx, self.name)
                if yaw is None:
                    node.get_logger().warn(
                        f"[{self.name}] Arm-sweep mode but no wall normal; falling "
                        f"back to the along-wall heading."
                    )
            if yaw is None:
                dx = other_point[0] - target_point[0]
                dy = other_point[1] - target_point[1]
                yaw = atan2(dy, dx)
            qz = sin(yaw / 2.0)
            qw = cos(yaw / 2.0)

            # Base goal. In phase 2 the optimal-base service already gives a valid
            # base standoff (selected_base). In phase 1 the target is the EE scan
            # point (~0.6 m off the wall); driving the base there fails Nav2 because
            # it sits inside robot_radius/inflation, so push the base further out
            # along the exterior normal (-inward_normal) to a safe standoff.
            if ctx.get("scan_phase") == 2:
                goal_xy = ctx.get("selected_base")
            elif partition_goal is not None:
                goal_xy = (partition_goal[0], partition_goal[1])
            else:
                goal_xy = self._base_standoff_goal(ctx, target_point)

            # Construct PoseStamped goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_xy[0]
            goal_msg.pose.pose.position.y = goal_xy[1]
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw
            # No base-reversing recoveries (BackOutFromObstacle/BackUp) on FSM
            # goals: fail fast back to the FSM instead of backing off the wall.
            goal_msg.behavior_tree = nav_bt_xml(ctx, self.name)

            nav_client = ctx.get("nav_client", None)
            if nav_client is None:
                node.get_logger().error(f"[{self.name}] Navigation client not found.")
                ctx["error_triggered"] = True
                return
            
            self._goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
            self._goal_yaw = yaw
            node.get_logger().info(f"[{self.name}] Sending goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) with yaw {yaw:.2f} rad.")
            self._send_goal_future = nav_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
            self.waiting = True

        elif self.waiting and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().warn(f"[{self.name}] Navigation goal was rejected!")
                self.fail(ctx, "Nav2 rejected the navigation goal")
                return
            node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
            self._nav_goal_handle = goal_handle
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(lambda fut: self.result_callback(fut, ctx))

            self.waiting = False

        elif not self._nav_result_pending and self._arrived_but_still_navigating(ctx):
            # Nav2 is at the goal POSITION and can only be trying to rotate now --
            # and at the scan standoff it provably cannot (ARM_SWEEP_PLAN §9: DWB
            # needs footprint + inscribed = 1.3 m of clearance, the standoff is
            # necessarily less, so BaseObstacle rejects the pure rotations and
            # RotateToGoal rejects anything with translation; the sample set
            # empties). Left alone it burns six recovery cycles of costmap clears
            # and 5 s waits against an obstacle that is the wall and is not going
            # anywhere.
            #
            # Cancel now and let the in-place /cmd_vel rotation below do the part
            # that works. result_callback fires on CANCELED like any other status,
            # so verification is unchanged.
            node.get_logger().info(
                f"[{self.name}] Base is at the goal position; cancelling Nav2 rather "
                f"than letting it retry a rotation it cannot plan this close to the "
                f"wall. Finishing the heading over /cmd_vel."
            )
            self._nav_cancel_sent = True
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception as e:
                node.get_logger().warn(f"[{self.name}] Nav2 cancel failed: {e}")

        elif self._nav_result_pending:
            # Nav2 finished (any status). Success is judged by the ACTUAL base
            # pose, not the action status: the goal checker tolerance (~0.5 m)
            # lets "success" end misaligned, and conversely an aborted goal can
            # still have parked the base well within scanning tolerance. The
            # wall-parallel controller and the scan geometry need the standoff
            # pose, so keep correcting until it is actually reached.
            self._nav_result_pending = False
            pos_err, yaw_err = self._pose_error(ctx)
            pos_tol = float(ctx.get("nav_pos_tolerance", self.POS_TOL_M))
            yaw_tol = float(ctx.get("nav_yaw_tolerance", self.YAW_TOL_RAD))
            if pos_err is None:
                node.get_logger().warn(
                    f"[{self.name}] Cannot verify base pose (no odom/TF); trusting "
                    f"nav status {self._nav_status}."
                )
                self.navigation_done = True
                return
            if pos_err is not None and pos_err <= pos_tol and yaw_err > yaw_tol:
                self.set_activity(
                    ctx, "Spinning turret to align the base heading along the wall"
                )
            elif pos_err is not None:
                self.set_activity(
                    ctx, "Correcting base pose to reach the scan standoff point"
                )
            if pos_err <= pos_tol and yaw_err <= yaw_tol:
                node.get_logger().info(
                    f"[{self.name}] Base pose verified at standoff goal "
                    f"(pos err {pos_err:.2f} m <= {pos_tol:.2f}, yaw err "
                    f"{yaw_err:.2f} rad <= {yaw_tol:.2f}). Navigation done."
                )
                self.navigation_done = True
                return
            # Residual error below Nav2's own goal tolerance cannot be fixed by
            # re-sending the goal (Nav2 "succeeds" instantly without moving), so
            # close-range errors could be servoed out directly over /cmd_vel — the
            # base is omnidirectional, a straight strafe+rotate is safe this
            # close to the verified-free standoff cell. Disabled by default
            # (fine_correction_enabled) because the open-loop /cmd_vel servo was
            # seen oscillating (overshoot past the goal and back); when off,
            # sub-tolerance misses fall through to the Nav2 goal retry below.
            # Position already good, only the heading is off. Rotate in place over
            # /cmd_vel rather than re-sending a Nav2 goal, because DWB CANNOT do
            # this near a wall: with footprint radius 0.7 m and an inscribed
            # radius of 0.6 m, any pose closer than ~1.3 m to the wall puts the
            # footprint over inscribed cells, so BaseObstacle rejects every
            # sample while RotateToGoal rejects anything with translation --
            # "No valid trajectories out of 6656". The scan standoff is
            # necessarily inside that band (the arm has to reach the wall), so
            # this is the normal case, not an edge case.
            #
            # Safe despite DWB's refusal: the base-only footprint is a CIRCLE, so
            # rotating about its centre sweeps no new area. DWB's check is static
            # (footprint vs inscribed cells), not a swept-volume test, so it is
            # merely conservative here. Translation is NOT safe on the same
            # argument, which is why this is rotation-only and the full x/y servo
            # stays behind fine_correction_enabled (it was seen oscillating).
            rotate_only = (
                pos_err <= pos_tol
                and bool(ctx.get("fine_correction_rotate_only", True))
            )
            servo_enabled = bool(ctx.get("fine_correction_enabled", False))
            max_servo_err = float(ctx.get("fine_correction_max_err", 0.7))
            if rotate_only:
                node.get_logger().info(
                    f"[{self.name}] Position within tolerance but heading is off "
                    f"(yaw err {yaw_err:.2f} rad > {yaw_tol:.2f}); rotating in place "
                    f"over /cmd_vel (Nav2 cannot rotate this close to the wall)."
                )
                self._start_fine_correction(ctx, rotate_only=True)
                return
            if servo_enabled and pos_err <= max_servo_err:
                node.get_logger().info(
                    f"[{self.name}] Fine-correcting base pose over /cmd_vel "
                    f"(pos err {pos_err:.2f} m, yaw err {yaw_err:.2f} rad)."
                )
                self._start_fine_correction(ctx, rotate_only=False)
                return
            max_retries = int(ctx.get("nav_pose_max_retries", self.MAX_POSE_RETRIES))
            if self._pose_retries < max_retries:
                self._pose_retries += 1
                node.get_logger().warn(
                    f"[{self.name}] Base pose far from goal (pos err {pos_err:.2f} m, "
                    f"yaw err {yaw_err:.2f} rad; nav status {self._nav_status}); nav "
                    f"retry {self._pose_retries}/{max_retries}."
                )
                self.goal_sent = False  # resend the goal (standoff recomputed)
                # A retry is a fresh goal: re-arm the arrival cancel, or the new
                # goal would run its full recovery budget after the first one used
                # the cancel up.
                self._nav_goal_handle = None
                self._nav_cancel_sent = False
                return
            node.get_logger().error(
                f"[{self.name}] Base pose still off goal after {max_retries} "
                f"correction(s) (pos err {pos_err:.2f} m, yaw err {yaw_err:.2f} rad). "
                f"Not starting the scan from a bad pose."
            )
            self.fail(
                ctx,
                f"base never reached the standoff pose (off by {pos_err:.2f} m / "
                f"{yaw_err:.2f} rad after {max_retries} retries)",
            )

    # EE scan standoff baked into the scan line by _build_wall_data (offset=0.6).
    SCAN_OFFSET_M = 0.2
    # Default base-center distance from the wall face (> robot_radius+inflation).
    DEFAULT_BASE_STANDOFF_M = 1.0
    # Pose-verification gate before hand-off to ArmUnfolding/WallParallel: the
    # base must actually be at the standoff pose, not just "nav reported done".
    # ctx/ROS params nav_pos_tolerance / nav_yaw_tolerance / nav_pose_max_retries
    # override these defaults.
    POS_TOL_M = 0.30
    YAW_TOL_RAD = 0.25
    MAX_POSE_RETRIES = 3

    def _clamp_target_to_segments(self, ctx, wall_data, target_point):
        """Clamp the scan start to the reachable sub-segments of the wall.

        Returns the (possibly moved) target point, or None when the whole wall
        is unreachable (error_triggered is set). Also publishes the segment
        markers and stores the segments in ctx for ScanWall to sweep.
        """
        node = ctx["node"]
        segments = reachable_wall_segments(ctx, wall_data[0], wall_data[1])
        if segments is None:
            return target_point  # no costmap yet; await_costmap gated us, keep raw
        if not segments:
            node.get_logger().error(
                f"[{self.name}] No portion of the wall is reachable within arm "
                f"reach (base_goal_max_offset); cannot scan this wall."
            )
            ctx["error_triggered"] = True
            return None
        ctx["current_wall_segments"] = segments
        publish_wall_segment_markers(ctx, segments)
        node.get_logger().info(
            f"[{self.name}] Wall split into {len(segments)} reachable segment(s): "
            + "; ".join(
                f"({s[0]:.2f}, {s[1]:.2f})->({e[0]:.2f}, {e[1]:.2f})"
                for s, e in segments
            )
        )
        # Nearest segment endpoint to the requested start (segments lie on the
        # scan line, so endpoints are the only candidates for a sweep start).
        tx, ty = float(target_point[0]), float(target_point[1])
        best = min(
            (pt for seg in segments for pt in seg),
            key=lambda p: (p[0] - tx) ** 2 + (p[1] - ty) ** 2,
        )
        moved = math.hypot(best[0] - tx, best[1] - ty)
        if moved > 1e-3:
            node.get_logger().info(
                f"[{self.name}] Scan start clamped to reachable segment endpoint: "
                f"({tx:.2f}, {ty:.2f}) -> ({best[0]:.2f}, {best[1]:.2f}) "
                f"({moved:.2f} m inset)."
            )
            ctx["target_scan_point"] = best
        return best

    def _first_partition_pose(self, ctx, wall_data, target_point):
        """Base scan pose of the first partition ScanWall will sweep, or ``None``.

        ``None`` means "no partition plan available, use the legacy wall-start
        goal" -- either the costmap has not arrived (clamping was skipped too, so
        the raw target is all anyone has) or no partition could be placed, which
        :func:`plan_wall_partitions` has already explained in the log. Falling
        back costs an extra ScanWall transit; it never scans the wrong place.

        The near/far ordering comes from :func:`sweep_line_order`, the same
        helper ScanWall uses, so partition 1 here IS partition 1 there.
        """
        node = ctx["node"]
        near_end, far_end = sweep_line_order(wall_data, target_point)
        plan = plan_wall_partitions(ctx, self.name, near_end, far_end)
        if plan is None:
            node.get_logger().warn(
                f"[{self.name}] No costmap yet to partition the wall; approaching "
                f"its reachable start instead (ScanWall will transit to partition 1)."
            )
            return None
        partitions, poses = plan
        if not poses:
            return None      # plan_wall_partitions logged the reason

        (px, py, _), (qx, qy, _) = partitions[0]
        node.get_logger().info(
            f"[{self.name}] Approaching partition 1/{len(poses)} directly: sweep "
            f"({px:.2f}, {py:.2f})->({qx:.2f}, {qy:.2f}), base scan pose "
            f"({poses[0][0]:.2f}, {poses[0][1]:.2f}) at yaw {poses[0][2]:.2f} rad."
        )
        return poses[0]

    # Fine-correction servo (last few decimeters, below Nav2's goal tolerance).
    SERVO_RATE_HZ = 10.0
    SERVO_KP_LIN = 0.6
    SERVO_KP_ANG = 0.9
    SERVO_MAX_LIN = 0.12   # m/s
    SERVO_MAX_ANG = 0.35   # rad/s
    SERVO_TIMEOUT_S = 30.0

    def _start_fine_correction(self, ctx, rotate_only=False):
        """Servo the omni base onto the standoff pose over /cmd_vel (through the
        twist_mux navigation channel; no Nav2 goal is active meanwhile). Ends by
        setting navigation_done (verified) or error_triggered (timeout).

        ``rotate_only`` commands yaw alone and holds translation at zero — the
        safe case near a wall (a circular footprint rotating about its centre
        sweeps no new area) and the one that cannot oscillate in position.
        """
        node = ctx["node"]
        if ctx.get("_cmd_vel_pub") is None:
            ctx["_cmd_vel_pub"] = node.create_publisher(Twist, "/cmd_vel", 10)
        self._stop_fine_correction(ctx)  # no duplicate timers
        self._servo_rotate_only = bool(rotate_only)
        self._servo_start = time.time()
        self._servo_timer = node.create_timer(
            1.0 / self.SERVO_RATE_HZ, lambda: self._fine_correction_tick(ctx)
        )

    def _stop_fine_correction(self, ctx, publish_stop=False):
        timer = getattr(self, "_servo_timer", None)
        if timer is not None:
            timer.cancel()
            ctx["node"].destroy_timer(timer)
            self._servo_timer = None
        if publish_stop and ctx.get("_cmd_vel_pub") is not None:
            ctx["_cmd_vel_pub"].publish(Twist())  # zero twist

    def _fine_correction_tick(self, ctx):
        node = ctx["node"]
        pos_err, yaw_err = self._pose_error(ctx)
        pos_tol = float(ctx.get("nav_pos_tolerance", self.POS_TOL_M))
        yaw_tol = float(ctx.get("nav_yaw_tolerance", self.YAW_TOL_RAD))
        if pos_err is None:
            self._stop_fine_correction(ctx, publish_stop=True)
            node.get_logger().error(f"[{self.name}] Lost base pose during fine correction.")
            ctx["error_triggered"] = True
            return
        # Rotation-only converges on yaw alone. turret_footprint (the frame Nav2
        # and _pose_error use) sits ~0.23 m off the chassis rotation centre, so a
        # pure chassis rotation traces it around a small circle -- demanding the
        # position tolerance too could leave this spinning until it times out.
        rotate_only = self._servo_rotate_only
        converged = yaw_err <= yaw_tol and (rotate_only or pos_err <= pos_tol)
        if converged:
            self._stop_fine_correction(ctx, publish_stop=True)
            if rotate_only and pos_err > pos_tol:
                # The rotation walked the base off the standoff point. Hand back
                # to the retry path rather than accepting a bad scan pose.
                node.get_logger().warn(
                    f"[{self.name}] Heading corrected (yaw err {yaw_err:.2f} rad) but "
                    f"the rotation moved the base off the standoff point "
                    f"(pos err {pos_err:.2f} m > {pos_tol:.2f}); re-verifying."
                )
                self._nav_result_pending = True
                return
            node.get_logger().info(
                f"[{self.name}] Base pose verified after fine correction "
                f"(pos err {pos_err:.2f} m, yaw err {yaw_err:.2f} rad). Navigation done."
            )
            self.navigation_done = True
            return
        timeout = float(ctx.get("fine_correction_timeout", self.SERVO_TIMEOUT_S))
        if time.time() - self._servo_start > timeout:
            self._stop_fine_correction(ctx, publish_stop=True)
            node.get_logger().error(
                f"[{self.name}] Fine correction timed out after {timeout:.0f}s "
                f"(pos err {pos_err:.2f} m, yaw err {yaw_err:.2f} rad)."
            )
            ctx["error_triggered"] = True
            return

        # P-servo: map-frame error rotated into the base frame (Twist is body-frame).
        pose = self._base_pose_map(ctx)
        if pose is None:
            self._stop_fine_correction(ctx, publish_stop=True)
            node.get_logger().error(f"[{self.name}] Lost base pose during fine correction.")
            ctx["error_triggered"] = True
            return
        px, py, yaw = pose
        ex, ey = self._goal_xy[0] - px, self._goal_xy[1] - py
        ex_b = cos(yaw) * ex + sin(yaw) * ey
        ey_b = -sin(yaw) * ex + cos(yaw) * ey
        dyaw = atan2(sin(self._goal_yaw - yaw), cos(self._goal_yaw - yaw))

        def clamp(v, lim):
            return max(-lim, min(lim, v))

        cmd = Twist()
        if not self._servo_rotate_only:
            cmd.linear.x = clamp(self.SERVO_KP_LIN * ex_b, self.SERVO_MAX_LIN)
            cmd.linear.y = clamp(self.SERVO_KP_LIN * ey_b, self.SERVO_MAX_LIN)
        cmd.angular.z = clamp(self.SERVO_KP_ANG * dyaw, self.SERVO_MAX_ANG)
        ctx["_cmd_vel_pub"].publish(cmd)

    # Base frames tried (in order) when looking up the map-frame base pose.
    # Fallback frame order when nav_base_frame is unavailable. turret_footprint
    # first: the omni Nav2 config drives that frame, so it must win over base_link.
    BASE_FRAMES = ("turret_footprint", "base_footprint", "base_link", "base", "chassis")

    def _base_pose_map(self, ctx):
        """Current base pose (x, y, yaw) in the map frame from TF, or None.

        The Nav2 goal is expressed in the map frame, so verification must read
        the base pose in the map frame too. ctx["base_position"] comes from
        /rtabmap/odom (the odom frame); comparing it against a map-frame goal is
        off by the whole map->odom correction and makes the gate fail even when
        Nav2 genuinely reached the goal, so use TF and never fall back to odom.

        Read the SAME frame Nav2 uses as its robot_base_frame (nav2_params_omni:
        turret_footprint). turret_footprint is the ground-projected *turret*
        heading and rotates independently of base_link, so measuring base_link
        instead adds the turret's mount offset + rotation to every error (the
        ~90 deg yaw miss and the extra XY residual that push the gate to fail
        even though Nav2 genuinely reached the goal). Override the frame with the
        nav_base_frame ctx/ROS param to match the launched Nav2 config.
        """
        tf_buffer = ctx.get("tf_buffer")
        if tf_buffer is None:
            return None
        primary = str(ctx.get("nav_base_frame", "turret_footprint"))
        frames = (primary,) + tuple(f for f in self.BASE_FRAMES if f != primary)
        for base_frame in frames:
            try:
                if not tf_buffer.can_transform(
                    "map", base_frame, rclpy.time.Time(), Duration(seconds=0.2)
                ):
                    continue
                tf = tf_buffer.lookup_transform(
                    "map", base_frame, rclpy.time.Time(), Duration(seconds=0.5)
                )
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return float(t.x), float(t.y), yaw
        return None

    def _arrived_but_still_navigating(self, ctx):
        """True when a Nav2 goal is in flight and the base has already reached its
        POSITION, so nothing Nav2 does next can help.

        Deliberately position-only: the heading is the part Nav2 cannot finish
        here, and it is exactly the part the in-place servo handles. Guarded by
        ``nav_cancel_on_arrival`` in case a deployment wants Nav2 to run its full
        course.
        """
        if not bool(ctx.get("nav_cancel_on_arrival", True)):
            return False
        if self._nav_cancel_sent or self._nav_goal_handle is None or self.waiting:
            return False
        pos_err, _ = self._pose_error(ctx)
        if pos_err is None:
            return False
        return pos_err <= float(ctx.get("nav_pos_tolerance", self.POS_TOL_M))

    def _pose_error(self, ctx):
        """(position error m, |yaw error| rad) between the current base pose and
        the last sent goal, or (None, None) when no pose source is available."""
        if self._goal_xy is None:
            return None, None
        pose = self._base_pose_map(ctx)
        if pose is None:
            return None, None
        x, y, yaw = pose
        pos_err = math.hypot(x - self._goal_xy[0], y - self._goal_xy[1])
        yaw_err = abs(atan2(sin(yaw - self._goal_yaw), cos(yaw - self._goal_yaw)))
        return pos_err, yaw_err

    def _base_standoff_goal(self, ctx, target_point):
        """Phase-1 base goal near the wall the base can actually reach.

        The EE scan point sits ~0.6 m off the wall, inside the Nav2 costmap's
        inflated band, so the global planner rejects it. Push it outward along
        the exterior normal (-inward_normal) to the nearest costmap cell the base
        can occupy; if the costmap is unavailable, fall back to a fixed standoff;
        if the inward normal is missing, use the raw scan point (old behaviour).
        """
        return base_standoff_goal(ctx, self.name, target_point,
                                  self.DEFAULT_BASE_STANDOFF_M, self.SCAN_OFFSET_M)

    def goal_response_callback(self, future):
        pass

    def result_callback(self, future, ctx):
        node = ctx["node"]
        status = future.result().status

        node.get_logger().info(
            f"[{self.name}] Navigation finished (status={status}); verifying base pose."
        )
        # Arrived at (or near) the base goal: drop the arm from the Nav2 footprint
        # so the arm-inclusive hull crossing the wall can't block the final
        # approach / fine-correction / nav retries. The arm is folded here, so a
        # base-only circle is the true collision envelope anyway.
        set_arm_footprint_enabled(ctx, False)
        # Verification happens in run() (single-threaded executor tick), where
        # the goal can be re-sent if the base is off the standoff pose.
        self._nav_status = status
        self._nav_result_pending = True
        return

    # def goal_response_callback(self, future: Future, node):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         node.get_logger().warn(f"[{self.name}] Goal rejected by server.")
    #         self.result_received = True
    #         self.last_status = goal_handle.status
    #         return

    #     node.get_logger().info(f"[{self.name}] Goal accepted. Waiting for result...")
    #     self.result_future = goal_handle.get_result_async()
    #     self.result_future.add_done_callback(lambda f: self.result_callback(f, node))

    # def result_callback(self, future: Future, node):
    #         result = future.result()
    #         self.last_status = result.status
    #         self.result_received = True
    #         if self.last_status == 4:
    #             node.get_logger().info(f"[{self.name}] Goal reached successfully.")
    #         else:
    #             node.get_logger().warn(f"[{self.name}] Goal failed with status {self.last_status}.")

    def check_transition(self, ctx):
        if self.navigation_done:
            return "ArmUnfolding"
        if ctx.get("error_triggered"):
            return "Error"
        return None
