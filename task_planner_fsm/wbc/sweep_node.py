#!/usr/bin/env python3
"""Whole-body compliant sweep of one wall segment.

Replaces the ``sweep_wait`` phase of ``ScanWall`` — a Nav2 position goal for the
base plus ``wall_parallel_controller`` streaming IK setpoints to the arm, two
controllers each unaware of the other — with ONE control law over base + arm::

    read the six plate ranges -> sensed surface frame (normal, tangent)
    build the task twist:  sweep along the tangent
                           hold the standoff along the normal
                           hold the row height
                           keep the plate parallel and upright
    resolve it across 9 DOF with a QP that respects the base's actuator limits
    scale by the robot's execution speed, bound the acceleration, and publish
    base Twist (turret frame)  +  arm command (servoj setpoints by default)

Started per segment by the FSM with the segment endpoints as parameters, it
reports on ``status_topic`` ("running" / "succeeded" / "failed: <reason>") and
the FSM's ``sweep_wait`` waits on that exactly as it waits on a Nav2 result.
Termination is by ARC LENGTH along the sensed surface, not by a base pose: the
sweep is done when the plate has travelled the segment.

The column is not touched — it holds the row height set before the sweep. On the
real robot ``force_mode`` cannot coexist with a streaming controller, so the
press is still sim-only and the FSM gates the sweep accordingly; everything
between the QP and the actuators, though, is written for the hardware — see
``wbc/streaming.py`` (why the arm gets positions, not velocities) and
``wbc/hardware.py`` (why the base is scaled by the UR's speed slider).

Run standalone (outside the FSM) for bench tests::

    ros2 run task_planner_fsm wbc_sweep_controller --ros-args \
      -p seg_start:="[1.0, 2.0, 1.2]" -p seg_end:="[3.0, 2.0, 1.2]" \
      -p sweep_speed:=0.03
"""

import math
import signal
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from .admittance import SEEK, TARE, AdmittancePress
from .avoidance import AvoidanceConfig, ObstacleField, avoidance_rows
from .base_model import BaseLimits, box_bounds, constraint_rows, wheel_and_turret_rates
from .hardware import HardwareMonitor
from .kinematics import SerialChain, rotation_error, whole_body_jacobian
from .qp import Task, joint_limit_bounds, solve_velocity_qp
from .streaming import DEFAULT_CONTROLLER, POSITION, ArmStream, slew_limit
from .surface import SurfaceEstimator, plate_orientation_target, sweep_tangent

ARM_JOINTS = [
    "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint",
    "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint",
]


class WholeBodySweepNode(Node):

    def __init__(self, **kwargs):
        # kwargs are passed through to rclpy's Node so tests can inject
        # parameter_overrides without a launch file or a running robot.
        super().__init__("wbc_sweep_controller", **kwargs)

        # --- What to sweep ---------------------------------------------------
        # Segment endpoints are ON the wall, in the map frame, at the row height
        # (the same points the FSM's reachable_wall_segments produces).
        self.declare_parameter("seg_start", [0.0, 0.0, 0.0])
        self.declare_parameter("seg_end", [0.0, 0.0, 0.0])
        self.declare_parameter("row_z", float("nan"))     # NaN -> hold the height we start at
        self.declare_parameter("sweep_speed", 0.03)       # m/s along the wall
        self.declare_parameter("standoff", 0.20)          # m, plate to wall
        self.declare_parameter("arrive_tolerance", 0.03)  # m of arc length
        self.declare_parameter("timeout_pad", 30.0)       # s beyond length/speed
        # Where to leave the plate when the sweep ends, so the base can transit
        # with the arm clear of the wall. Match the FSM's
        # scan_wall_transit_plate_offset. 0 disables the retreat.
        self.declare_parameter("retreat_standoff", 0.40)
        self.declare_parameter("retreat_speed", 0.05)     # m/s along the normal
        self.declare_parameter("retreat_timeout", 20.0)   # s on the node clock
        # Arm configuration to finish in, normally the unfolded pose the FSM's
        # planner last placed the arm in. Retreating along the normal alone
        # cannot leave the arm somewhere the planner will accept: too little and
        # A* dies on the wall's dilation, too much and wrist_3 enters the mast's
        # keep-out cylinder, and the safe window moves with the base's standoff.
        # Empty disables the return.
        # Declared with an explicit DOUBLE_ARRAY descriptor, NOT `[]`: rclpy
        # infers the element type from the default, and an empty list gives it
        # nothing to infer, so it settles on BYTE_ARRAY and then rejects the
        # override at startup — the node dies before it can report anything.
        self.declare_parameter(
            "return_joints", None,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter("return_gain", 1.0)        # 1/s toward the target
        self.declare_parameter("return_tolerance", 0.05)  # rad, per joint
        self.declare_parameter("return_timeout", 25.0)    # s on the node clock
        # Fail a sweep that has stopped advancing this long, instead of waiting
        # out the whole length/speed budget with the plate stuck on something.
        self.declare_parameter("no_progress_timeout", 25.0)

        # --- Frames and I/O --------------------------------------------------
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "turret_footprint")
        # The chain root doubles as the TF frame for the arm mount, so the model
        # and the measurement can never disagree about which frame that is. Note
        # it is 'arm_base_link' (the URDF link), NOT the FSM's 'arm_base' — the
        # UR description carries both, rotated pi about z from each other.
        self.declare_parameter("arm_root_link", "arm_base_link")
        # The plate ranges are measured at 'arm_plate_link', which shares its
        # orientation with 'arm_tool0' (both hang off arm_ee_cylinder_link with
        # the same rpy) and sits 0.15 m further along the shared +Z. Using tool0
        # as the tip therefore keeps the sensed normal valid, and the standoff
        # keeps the same meaning it has everywhere else in the FSM.
        self.declare_parameter("arm_tip_link", "arm_tool0")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # How the arm is commanded: "position" streams servoj setpoints through
        # forward_position_controller, "velocity" streams speedj through
        # forward_velocity_controller. Position is the default because losing
        # this node then freezes the arm instead of running it away — see
        # wbc/streaming.py. Both topic and controller default to whatever the
        # chosen interface implies; "" means "derive it".
        self.declare_parameter("arm_stream_interface", POSITION)
        self.declare_parameter("arm_command_topic", "")
        self.declare_parameter("arm_stream_controller", "")
        # How far the integrated setpoint may lead the measured joint before it
        # is clamped. Sized off the servo lag at these speeds (~0.5 rad/s into a
        # ~30 ms lag is ~0.015 rad), so this is an order of magnitude of
        # headroom and only bites when the arm is genuinely not following.
        self.declare_parameter("arm_stream_max_lead", 0.2)
        # --- The GPR press (real robot only) ----------------------------------
        # Regulate CONTACT FORCE on the wall-normal axis instead of a standoff
        # distance, so the GPR wheel actually touches. This is our own admittance
        # loop because UR's force_mode cannot run alongside a streaming
        # controller — confirmed on hardware, see wbc/admittance.py. Off by
        # default: there is no force/torque sensor in Gazebo (the <sensor> block
        # in arm_control's ur.ros2_control.xacro is inside an
        # `unless sim_gazebo or sim_ignition`), and a sweep that waits for a
        # contact force that can never arrive would just stall.
        self.declare_parameter("press_enabled", False)
        self.declare_parameter("wrench_topic", "/force_torque_sensor_broadcaster/wrench")
        self.declare_parameter("press_force", 5.0)          # N, matches the old force_mode
        # m/s per N. Small on purpose: the loop is a velocity source against a
        # stiff environment, so k * K_e sets the closed-loop bandwidth and must
        # stay well under the servo lag. See the module docstring for the sizing.
        self.declare_parameter("press_gain", 5.0e-5)
        self.declare_parameter("press_v_max", 0.005)        # m/s
        self.declare_parameter("press_seek_speed", 0.01)    # m/s, closing on the wall
        self.declare_parameter("press_contact_force", 1.0)  # N, SEEK -> PRESS
        self.declare_parameter("press_release_force", 0.5)  # N, PRESS -> SEEK
        self.declare_parameter("press_force_limit", 25.0)   # N, abort above this
        # The distance sensors stop being the setpoint and become the envelope:
        # no approach closer than this to the sensed plane, whatever the force
        # says. A wrong force reading then cannot walk the arm into the wall,
        # because a different sensor is what stops it.
        #
        # Sized off the HARDWARE STANDOFF, not off zero. The six range sensors
        # sit on the plate face, but the GPR body has length along the wall
        # normal and four bars with caster wheels stand off each corner, so when
        # the wheel and the casters are all riding the wall the plate is still
        # ~0.13 m off it and that is what the ranges read. The plate BOTTOMS OUT
        # there — measured on hardware, it cannot physically get closer.
        #
        # So an envelope near zero can never be crossed and protects nothing,
        # which is exactly what the old 0.005 did. What it must catch is the
        # plate being driven PAST its own stop, i.e. something deforming because
        # a bad force reading kept SEEK pushing. That means sitting just below
        # 0.13, with enough room for sensor noise.
        #
        # Mind that room, because the ranges are noisy. The per-sensor sigma is
        # 0.010 m and the plane fit over six of them measures out at 4.2 mm
        # sigma on the fitted distance, so with the plate resting quietly at its
        # stop the envelope still trips on its own:
        #
        #     0.125 -> 12.4% of cycles      0.115 -> 0.02%
        #     0.120 ->  1.0% of cycles      0.110 -> never
        #
        # 0.115 is the value that buys a real guard without crying wolf. It sits
        # 1.5 cm inside the stop, which is far more travel than the bars have in
        # them, and about 3.5 sigma clear of the noise.
        #
        # A trip is cheap either way: it clamps the approach to zero for that
        # one cycle, and the stall counter resets on any untripped cycle, so
        # noise alone can never reach the 100 in a row that would fail the
        # sweep. What a too-tight envelope costs is a press that sits light.
        self.declare_parameter("press_min_distance", 0.115)  # m
        self.declare_parameter("press_filter_alpha", 0.2)
        self.declare_parameter("press_stall_cycles", 100)
        # Tare: hold the normal axis still for this many cycles at the start and
        # average what the sensor reads in free space, then subtract it. 25 is
        # half a second at 50 Hz. Set to 0 to trust the sensor as it comes, which
        # is only sane if something else has just tared it. The plate must be at
        # least press_tare_min_distance off the surface, or the tare would fold
        # the contact force into the zero — see wbc/admittance.py.
        self.declare_parameter("press_tare_cycles", 25)
        self.declare_parameter("press_tare_min_distance", 0.05)   # m
        # How long to wait for the wheel to reach the wall before giving up on
        # the segment. The base holds still for all of it (see _control_step),
        # so this is not a stall — but it has to be bounded, because a press that
        # never arrives would otherwise hold the base for the whole sweep budget
        # and then report a segment it never scanned. Generous: the approach is
        # accepted anywhere within scan_wall_approach_tolerance, so the gap can
        # be up to ~0.22 m, which is 22 s at press_seek_speed.
        self.declare_parameter("press_contact_timeout", 45.0)     # s
        # How hard the normal axis is held to what the force loop asks for. Large
        # because a press command is orders of magnitude smaller than the sweep
        # travel, and anything less lets it disappear into the pooled task's
        # residual. See where it is used for the measurement behind that.
        self.declare_parameter("weight_press_normal", 1.0e4)
        self.declare_parameter("distance_topic", "/distance_sensors")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("status_topic", "/wbc_sweep/status")
        # Per-cycle trace, for telling WHERE a jerky motion comes from instead
        # of guessing. Off by default: it is a message per control cycle, and
        # nothing in the loop reads it back. Turn it on for a diagnostic run,
        # record it, turn it off again.
        #
        #     ros2 run task_planner_fsm wbc_sweep_controller --ros-args \
        #       -p publish_diagnostics:=true
        #     ros2 bag record /wbc_sweep/diagnostics
        #
        # The three traces that matter are the solver's own answer, the command
        # actually published, and the velocity the arm actually reached. Read in
        # that order they localise the fault: a solve that is already stepped is
        # a control-law problem, a smooth solve with a stepped command is the
        # streaming layer, and smooth on both with a rough arm is the driver,
        # the servo tuning or the speed scaling. Plot all three before changing
        # anything, rather than watching the robot and guessing.
        self.declare_parameter("publish_diagnostics", False)
        self.declare_parameter("diagnostics_topic", "/wbc_sweep/diagnostics")
        self.declare_parameter("turret_joint", "turret_joint")
        self.declare_parameter("arm_joints", ARM_JOINTS)

        # --- Control law -----------------------------------------------------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("k_standoff", 1.0)      # 1/s on the normal error
        self.declare_parameter("k_height", 1.0)        # 1/s on the row-height error
        self.declare_parameter("k_align", 1.5)         # 1/s on the plate orientation error
        self.declare_parameter("v_normal_max", 0.05)   # m/s cap on the standoff correction
        self.declare_parameter("w_align_max", 0.30)    # rad/s cap on the orientation correction
        self.declare_parameter("ema_alpha", 0.3)       # surface-normal low-pass
        self.declare_parameter("weight_linear", 1.0)
        self.declare_parameter("weight_angular", 0.5)
        # Per-DOF damping. The arm is damped ~10x harder than the base so the
        # redundancy resolves the way the task wants it to: the BASE carries the
        # along-wall travel (it can go forever) and the arm spends its motion on
        # the standoff, height and parallelism corrections it is quick at.
        self.declare_parameter("damping_base", [0.01, 0.01, 0.02])
        # Keep this a regulariser, not a competing objective. At 0.1 the cost of
        # a 0.5 rad/s arm motion is the same order as the task cost of missing
        # the reference by 0.05 m/s, so once the base is (correctly) discouraged
        # from taking normal corrections, the QP simply declines to close the
        # standoff error at all. It only looked fine while the base was doing the
        # arm's job for it.
        self.declare_parameter("damping_arm", 0.03)
        # How hard the base is discouraged from moving along the wall normal, so
        # the arm takes the standoff corrections instead. Costs the sweep nothing
        # (the sweep is tangential) but is a soft term, not a constraint, so the
        # base can still follow a wall that is not straight.
        self.declare_parameter("weight_base_normal", 100.0)
        # Temporal regularisation: how hard the solve is pulled toward the
        # answer it gave last cycle. This is the term that makes the SOLUTION
        # continuous rather than truncating its jumps afterwards.
        #
        # The QP is memoryless. Its active set moves between cycles — a barrier
        # engages, a joint limit releases, the travel cap tightens — and nothing
        # in the cost function has ever said that two consecutive answers ought
        # to resemble each other, so they need not. That is a step at the
        # actuator, and on a setpoint stream it reads as the arm moving and
        # stopping in short hops.
        #
        # Distinct from damping, which is already here and does a different job:
        # damping penalises ||u||, so it pulls toward STOPPING. This penalises
        # ||u - u_prev||, so it pulls toward CARRYING ON. A sweep wants the
        # second one; only the first was present.
        #
        # Small, and the ceiling is MEASURED rather than guessed. Do not raise
        # it on the strength of how much smoother it makes a clear wall look.
        #
        # Sweeping a flat wall, more of this is free all the way up. Worst arm
        # step falls 0.021 -> 0.018 -> 0.012 -> 0.003 rad/s at weights of 0,
        # 0.05, 0.5 and 5.0, while the standoff error sits at 0.048 m the whole
        # way. On that evidence alone the weight belongs an order of magnitude
        # higher than it is.
        #
        # That evidence is a trap, because a flat wall never asks the solve to
        # change its mind. Put an obstacle across the base's path, where the
        # barrier's whole job is to bring the base to a STOP, and preferring
        # last cycle's answer is precisely the wrong instinct:
        #
        #     weight   base travel when the barrier demands a stop
        #     0.0        0.000000   stops
        #     0.05       0.000000   stops
        #     0.1        0.006773   KEEPS MOVING
        #     0.5        0.024890   KEEPS MOVING
        #
        # The barrier rows go in soft so they can never make the solve
        # infeasible, which means they win on weight or not at all, and past
        # 0.05 this term outvotes them. So this is not a comfort setting to be
        # turned up until the motion looks nice: it is bounded above by the
        # obstacle avoidance still working, and 0.05 is the last value that
        # leaves zero reachable.
        self.declare_parameter("weight_smoothness", 0.05)
        self.declare_parameter("weight_posture", 0.02)
        self.declare_parameter("k_posture", 0.5)
        # Ceiling on how fast the posture task may pull, in rad/s. Without it the
        # posture pull grows with joint displacement while the standoff reference
        # is saturated at v_normal_max, so the secondary task eventually
        # overpowers the primary one and the standoff settles with a large steady
        # error. A background term must stay in the background.
        self.declare_parameter("posture_rate_max", 0.05)
        self.declare_parameter("arm_qdot_max", 0.5)     # rad/s
        self.declare_parameter("joint_limit_margin", 0.15)  # rad

        # --- Base actuator limits (mirror the sim_controller config) ---------
        self.declare_parameter("center_distance", 0.167)
        self.declare_parameter("wheel_radius", 0.125)
        self.declare_parameter("wheel_separation", 0.3655)
        self.declare_parameter("max_x_velocity", 0.2)
        self.declare_parameter("max_y_velocity", 0.2)
        self.declare_parameter("max_angular_turret", 0.35)
        self.declare_parameter("max_angular_base", 0.2)
        self.declare_parameter("max_turret_motor_speed", 0.5)
        self.declare_parameter("sweep_speed_margin", 0.9)   # of the computed cap
        # Pin the base's FORWARD velocity to the along-wall reference instead of
        # letting the QP apportion the travel between base and arm. Two reasons,
        # and they are both about the base being the wrong actuator for detail:
        #
        #   * The GPR line scan assumes a steady travel speed — it counts
        #     distance off its own wheel. A base velocity that dips whenever the
        #     solver's active set changes is exactly the input that smears a
        #     scan, and nothing downstream can undo it.
        #   * The base is the imprecise one: casters that must break away before
        #     they swivel, stiction at a few cm/s, odometry drift. Asking it for
        #     the fine corrections gives the delicate work to the worst actuator.
        #     The arm has six joints and millimetre resolution — let it absorb
        #     the error while the base does one steady thing per segment.
        #
        # Only the turret's forward axis is pinned; vy and wz stay free so the
        # solver can still correct heading drift over a long segment, held near
        # zero by weight_base_normal and damping_base rather than by a hard row.
        self.declare_parameter("base_constant_travel", True)
        # How closely the turret must point along the sweep before the pin is
        # applied, as |cos(angle)|. During a wall scan the turret faces ALONG the
        # wall with the wall on its left, so this is ~1.0 and the pin holds. If
        # the turret is turned well off the sweep direction the forward axis is
        # no longer the travel axis, and pinning it would command the wrong thing
        # — so hand the apportioning back to the solver instead. 0.5 is 60 deg.
        self.declare_parameter("base_travel_min_alignment", 0.5)

        # --- Obstacle avoidance ----------------------------------------------
        # The sweep drives the base itself, so nothing else is watching for
        # obstacles while it runs; these turn the local costmap into barrier rows
        # in the same QP. See wbc/avoidance.py.
        self.declare_parameter("avoid_obstacles", True)
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("avoid_safety_margin", 0.15)
        self.declare_parameter("avoid_influence", 1.0)
        self.declare_parameter("avoid_alpha", 1.0)
        self.declare_parameter("avoid_obstacle_cost", 100)
        self.declare_parameter("avoid_unknown_is_obstacle", False)
        self.declare_parameter("avoid_footprint_radius", 0.45)
        self.declare_parameter("avoid_footprint_offset", [0.0, 0.0])
        self.declare_parameter("avoid_n_samples", 8)
        self.declare_parameter("avoid_max_rows", 6)
        # Must exceed the costmap's inflation radius, or the scanned wall's
        # inflated ring survives the mask and pushes the base off the wall.
        self.declare_parameter("avoid_mask_halfwidth", 0.7)
        self.declare_parameter("avoid_mask_extension", 3.0)
        self.declare_parameter("avoid_slack_weight", 1e3)
        # A costmap that never arrives is a warning, not a failure: the sweep is
        # slow, deliberate motion over ground the FSM already vetted with
        # reachable_wall_segments, and refusing to scan because Nav2 is not
        # publishing would be its own kind of unsafe. Flip this to insist.
        # Floor on the base's own distance to the wall being scanned, measured
        # from the plate's sensed standoff rather than from any map. Nav2 models
        # this base as a 0.6 m disc, so a base closer than that to the wall is
        # "in collision" by its own reckoning and no transit will plan.
        self.declare_parameter("base_wall_min_gap", 0.75)
        self.declare_parameter("avoid_require_costmap", False)
        self.declare_parameter("costmap_max_age", 10.0)

        # --- Safety ----------------------------------------------------------
        # Per-DOF acceleration bound on the published command, in the same order
        # as u: [vx, vy, wz | arm joints]. The QP has no memory, so its solution
        # can step when the active set changes (a barrier engages, a joint limit
        # releases); streamed straight out that step lands on the actuators.
        # These are loose enough to be invisible in normal operation — the
        # reference velocities are tenths of a unit — and only bite on steps.
        self.declare_parameter("base_accel_max", [0.3, 0.3, 0.6])   # m/s^2, m/s^2, rad/s^2
        self.declare_parameter("arm_accel_max", 2.0)                # rad/s^2
        # The UR's execution speed (teach-pendant slider, safety reduced mode)
        # scales the ARM but not the base, which desynchronises a whole-body
        # command. Reading it lets the whole command be scaled together. Absent
        # in sim, where it reads 1.0. See wbc/hardware.py.
        self.declare_parameter("speed_scaling_topic",
                               "/speed_scaling_state_broadcaster/speed_scaling")
        self.declare_parameter("robot_mode_topic", "/io_and_status_controller/robot_mode")
        self.declare_parameter("safety_mode_topic", "/io_and_status_controller/safety_mode")
        # Below this the robot is effectively paused (slider at zero, or a stop
        # in progress) and the sweep holds rather than commanding a scaled-down
        # crawl. A pause longer than max_hold_seconds ends the sweep — which is
        # the right outcome: the FSM can retry, and sitting with the arm loaded
        # against a wall indefinitely is not an improvement.
        self.declare_parameter("min_speed_scaling", 0.05)
        self.declare_parameter("max_data_age", 1.0)        # s
        self.declare_parameter("max_standoff_error", 0.25)  # m before we call it lost
        self.declare_parameter("standoff_error_cycles", 5)  # ... for this many cycles
        # How long the robot may sit stopped waiting for usable inputs before the
        # sweep is called off. In SECONDS, not cycles: at 50 Hz a cycle budget
        # turns a routine TF hiccup (rtabmap republishing map->odom) into a
        # failed sweep a fifth of a second later.
        self.declare_parameter("max_hold_seconds", 3.0)

        p = self.get_parameter
        self.seg_start = np.array(p("seg_start").value, dtype=float)
        self.seg_end = np.array(p("seg_end").value, dtype=float)
        self.row_z = float(p("row_z").value)
        self.standoff = float(p("standoff").value)
        self.arrive_tolerance = float(p("arrive_tolerance").value)
        self.map_frame = str(p("map_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.arm_root_link = str(p("arm_root_link").value)
        self.arm_tip_link = str(p("arm_tip_link").value)
        self.turret_joint = str(p("turret_joint").value)
        self.arm_joints = list(p("arm_joints").value)
        self.control_rate = float(p("control_rate").value)
        self.max_data_age = float(p("max_data_age").value)
        self.max_standoff_error = float(p("max_standoff_error").value)
        self.standoff_error_cycles = int(p("standoff_error_cycles").value)
        self.max_hold_seconds = float(p("max_hold_seconds").value)
        self.base_constant_travel = bool(p("base_constant_travel").value)
        self.base_travel_min_alignment = float(p("base_travel_min_alignment").value)
        # What the forward bound did THIS cycle, for the log: pinned to the
        # reference, capped at it while a barrier works, or neither. It is the
        # thing to look at when a sweep is jerky — a bound that keeps coming off
        # says the geometry or the obstacle field is fighting it, not the solver.
        self.base_travel_pinned = False
        self.base_travel_capped = False

        self.limits = BaseLimits(
            center_distance=float(p("center_distance").value),
            wheel_radius=float(p("wheel_radius").value),
            wheel_separation=float(p("wheel_separation").value),
            vx_max=float(p("max_x_velocity").value),
            vy_max=float(p("max_y_velocity").value),
            wz_max=float(p("max_angular_turret").value),
            w_chassis_max=float(p("max_angular_base").value),
            w_turret_motor_max=float(p("max_turret_motor_speed").value),
        )
        # The along-wall speed is set by the base, not chosen: sweeping sideways
        # costs chassis yaw rate (w = v_lateral / center_distance). Asking for
        # more just makes sim_controller rescale the command, which silently
        # distorts the sweep, so clamp the reference here and say so.
        # How fast the base can sweep depends on the direction it sweeps in, and
        # that is not known until the surface is sensed — so the reference is
        # capped per cycle (see _control_step), and this only says what to expect.
        self.sweep_speed = float(p("sweep_speed").value)
        sideways = self.limits.max_lateral_speed() * float(p("sweep_speed_margin").value)
        if self.sweep_speed > sideways:
            self.get_logger().warn(
                f"sweep_speed {self.sweep_speed:.3f} m/s is above the {sideways:.3f} m/s "
                f"this base holds sweeping SIDEWAYS (d1 x chassis yaw rate), which is the "
                f"usual geometry with the chassis parked square to the wall. It will be "
                f"capped per cycle by the actual sweep direction."
            )

        segment_length = float(np.linalg.norm(self.seg_end[:2] - self.seg_start[:2]))
        self.segment_length = segment_length
        self.deadline = None
        # When the sweep began commanding, so the diagnostics carry a clock
        # that starts at the segment rather than at the epoch.
        self.start_stamp = None
        self.timeout = segment_length / max(self.sweep_speed, 1e-3) + float(p("timeout_pad").value)

        # --- State -----------------------------------------------------------
        self.chain = None
        self.urdf = None
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_stamp = None
        # Filled each cycle when diagnostics are on: how long the solve took,
        # and the answer it gave before the publish path touched it.
        self.solve_seconds = 0.0
        self.distances = None
        self.distance_stamp = None
        # Press force along the plate's own +Z, positive = pressing INTO the
        # wall. The UR reports the opposite sign on tool0 Z, and the flip happens
        # in _on_wrench so nothing downstream has to think about it.
        self.press_force = 0.0
        self.wrench_stamp = None
        # When the sweep started waiting for the wheel to reach the wall. None
        # means it is not waiting — either it has touched, or it is not pressing.
        self.press_wait_since = None
        self.surface = SurfaceEstimator(ema_alpha=float(p("ema_alpha").value))
        self.q_posture = None
        self.holding_since = None
        self.standoff_strikes = 0
        # The last published command, which the acceleration bound limits the
        # next one against, and when it went out, which is only used to notice a
        # loop that is not keeping up. None means "no history yet"; a stop sets
        # the command to ZERO rather than clearing it, so the cycle that resumes
        # a sweep ramps up instead of jumping (see _strike).
        self.u_prev = None
        self.command_stamp = None
        # The SOLVER's own previous answer, and when it produced it. Distinct
        # from u_prev, which is the published command and therefore already
        # multiplied by the robot's speed scaling. The acceleration bound inside
        # the QP has to be measured against the unscaled quantity the QP itself
        # works in: bounding this cycle's solution against a HALVED published
        # command would drag the solution down every cycle and converge on a
        # speed nobody asked for.
        self.u_qp_prev = None
        self.qp_stamp = None
        self.cycle_period = 0.0
        self.accel_max = np.concatenate((
            np.array(p("base_accel_max").value, dtype=float),
            np.full(len(self.arm_joints), float(p("arm_accel_max").value))))
        self.min_speed_scaling = float(p("min_speed_scaling").value)
        # "sweep" -> "retreat" (backing the plate off the wall) -> "done". The
        # terminal status is withheld until "done", because the FSM kills this
        # process as soon as it sees one.
        self.phase = "sweep"
        self.pending_status = None
        self.retreat_deadline = 0.0
        self.return_deadline = 0.0
        # A typed-but-unset parameter RAISES on .value rather than returning
        # None, so an absent return target has to be caught, not defaulted.
        try:
            raw = p("return_joints").value
        except rclpy.exceptions.ParameterUninitializedException:
            raw = None
        self.return_joints = [float(v) for v in (raw or [])]
        self.status = "running"
        self.control_timer = None
        self.progress = 0.0
        self.best_progress = 0.0
        self.progress_stamp = None   # set on the first cycle, not at construction

        self.avoid = bool(p("avoid_obstacles").value)
        self.avoid_config = AvoidanceConfig(
            safety_margin=float(p("avoid_safety_margin").value),
            influence=float(p("avoid_influence").value),
            alpha=float(p("avoid_alpha").value),
            obstacle_cost=int(p("avoid_obstacle_cost").value),
            unknown_is_obstacle=bool(p("avoid_unknown_is_obstacle").value),
            footprint_radius=float(p("avoid_footprint_radius").value),
            footprint_offset=tuple(p("avoid_footprint_offset").value),
            n_samples=int(p("avoid_n_samples").value),
            max_rows=int(p("avoid_max_rows").value),
            mask_halfwidth=float(p("avoid_mask_halfwidth").value),
            mask_extension=float(p("avoid_mask_extension").value),
        )
        self.costmap_msg = None
        self.costmap_stamp = None
        self.obstacle_field = None
        self._field_from = None      # the message the cached field was built from
        self.closest_obstacle = float("inf")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS I/O ---------------------------------------------------------
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, str(p("status_topic").value), latched)
        self.cmd_vel_pub = self.create_publisher(Twist, str(p("cmd_vel_topic").value), 10)
        self.diag_pub = (
            self.create_publisher(
                Float64MultiArray, str(p("diagnostics_topic").value), 10)
            if bool(p("publish_diagnostics").value) else None)
        self.arm_stream = ArmStream(
            self, self.arm_joints,
            mode=str(p("arm_stream_interface").value),
            topic=str(p("arm_command_topic").value) or None,
            max_lead=float(p("arm_stream_max_lead").value))
        self.arm_controller = (str(p("arm_stream_controller").value)
                               or DEFAULT_CONTROLLER[self.arm_stream.mode])
        self.hardware = HardwareMonitor(
            self,
            speed_scaling_topic=str(p("speed_scaling_topic").value),
            robot_mode_topic=str(p("robot_mode_topic").value),
            safety_mode_topic=str(p("safety_mode_topic").value))
        self.press = None
        if bool(p("press_enabled").value):
            self.press = AdmittancePress(
                target_force=float(p("press_force").value),
                gain=float(p("press_gain").value),
                v_max=float(p("press_v_max").value),
                seek_speed=float(p("press_seek_speed").value),
                contact_force=float(p("press_contact_force").value),
                release_force=float(p("press_release_force").value),
                force_limit=float(p("press_force_limit").value),
                min_distance=float(p("press_min_distance").value),
                filter_alpha=float(p("press_filter_alpha").value),
                stall_cycles=int(p("press_stall_cycles").value),
                tare_cycles=int(p("press_tare_cycles").value),
                tare_min_distance=float(p("press_tare_min_distance").value))
            self.create_subscription(
                WrenchStamped, str(p("wrench_topic").value), self._on_wrench, 10)
        self.create_subscription(
            String, str(p("robot_description_topic").value), self._on_robot_description, latched)
        self.create_subscription(
            JointState, str(p("joint_states_topic").value), self._on_joint_states, 10)
        self.create_subscription(
            Float32MultiArray, str(p("distance_topic").value), self._on_distances, 10)
        if self.avoid:
            self.create_subscription(
                OccupancyGrid, str(p("costmap_topic").value), self._on_costmap, 1)
        # Republished, not just latched: the FSM may subscribe after we start.
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f"Whole-body sweep: ({self.seg_start[0]:.2f}, {self.seg_start[1]:.2f}) -> "
            f"({self.seg_end[0]:.2f}, {self.seg_end[1]:.2f})  [{segment_length:.2f} m] "
            f"at {self.sweep_speed:.3f} m/s, standoff {self.standoff:.2f} m."
        )
        self.get_logger().info(
            f"Arm commanded by {self.arm_stream.mode} on '{self.arm_stream.topic}' "
            f"via '{self.arm_controller}'."
        )

    def _now(self):
        """Seconds on the NODE's clock — sim time when ``use_sim_time`` is set.

        Never ``time.time()``: Gazebo on a loaded machine runs well below
        real-time (0.2x is normal with rtabmap and Nav2 alongside), so a
        wall-clock staleness window would declare fresh sensor frames stale, and
        a wall-clock deadline would time a sweep out several times faster than
        the robot can travel it. Everything here — data ages, the sweep deadline,
        the control period — must tick on the same clock the robot moves on.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_robot_description(self, msg):
        if self.chain is not None:
            return
        self.urdf = msg.data
        try:
            self.chain = SerialChain.from_urdf(self.urdf, self.arm_root_link, self.arm_tip_link)
        except Exception as exc:
            self.get_logger().error(f"Cannot build the arm chain from the URDF: {exc}")
            return
        self.get_logger().info(
            f"Arm chain {self.arm_root_link} -> {self.arm_tip_link}: "
            f"{self.chain.joint_names}"
        )
        # The QP solves for the CHAIN's joints; the command goes out in the
        # streaming controller's joint order. If the two sets disagree the
        # commands would be silently zeroed, so refuse the sweep instead.
        missing = set(self.chain.joint_names) ^ set(self.arm_joints)
        if missing:
            self.get_logger().error(
                f"The chain's joints and the 'arm_joints' parameter disagree "
                f"({sorted(missing)}); the arm would receive no command. "
                f"Set arm_joints to the controller's joint list."
            )
            self.chain = None
            return
        # Give the integrator the joint limits, in the CONTROLLER's order — the
        # order the setpoint is published in. The chain's order is not
        # guaranteed to match, and clamping a setpoint against another joint's
        # limits would be worse than not clamping at all.
        lower, upper = self.chain.position_limits()
        by_name = dict(zip(self.chain.joint_names, zip(lower, upper)))
        self.arm_stream.set_position_limits(
            [by_name[name][0] for name in self.arm_joints],
            [by_name[name][1] for name in self.arm_joints])
        if list(self.chain.joint_names) != list(self.arm_joints):
            # Same joints, different order. Every path here reorders explicitly,
            # so this is handled — but it is worth saying out loud, because a
            # mis-ordered POSITION setpoint commands the arm to a pose nobody
            # asked for, where a mis-ordered velocity merely moves it wrongly.
            self.get_logger().warn(
                f"The URDF chain order {self.chain.joint_names} differs from the "
                f"controller's {self.arm_joints}; commands are reordered to match "
                f"the controller. Align 'arm_joints' with the controller's joint "
                f"list to remove the ambiguity."
            )

    def _on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = float(position)
        # Velocities are for the diagnostics only — nothing in the control law
        # reads them. They are the third of the three traces that say WHERE a
        # jerky motion is coming from: if the solve is already stepped the cause
        # is upstream of the streaming layer, and if the solve and the published
        # command are both smooth while this is not, it is the driver, the servo
        # tuning or the speed scaling. Guarded because velocity is optional in
        # the message and some publishers leave it empty.
        if len(msg.velocity) == len(msg.name):
            for name, velocity in zip(msg.name, msg.velocity):
                self.joint_velocities[name] = float(velocity)
        self.joint_stamp = self._now()

    def _on_distances(self, msg):
        if len(msg.data) == 6:
            self.distances = np.array(msg.data, dtype=float)
            self.distance_stamp = self._now()

    # The press tunables that may be changed WHILE the wheel is loaded, as
    # (parameter, AdmittancePress attribute). Deliberately not the whole set:
    # min_distance is the safety envelope and filter_alpha is part of what keeps
    # the loop stable, so neither should move mid-press. These five are the ones
    # worth reaching for on a bench, and the important one is the gain.
    PRESS_TUNABLES = (
        ("press_force", "target_force"),
        ("press_gain", "gain"),
        ("press_v_max", "v_max"),
        ("press_seek_speed", "seek_speed"),
        ("press_force_limit", "force_limit"),
    )

    def _refresh_press_tuning(self):
        """Pick up ``ros2 param set`` on the press, once per cycle.

        Tuning a force loop against concrete by restarting the FSM means every
        attempt begins by driving the wheel back into the wall at the gain that
        just misbehaved. Reading these live instead means a press that starts to
        ring can be backed off from another terminal while it runs::

            ros2 param set /wbc_sweep_controller press_gain 2.5e-5

        Changes are logged at WARN rather than silently applied: a force loop
        that was retuned mid-run is the first thing anyone reading the log
        afterwards needs to know.
        """
        for name, attr in self.PRESS_TUNABLES:
            value = float(self.get_parameter(name).value)
            current = getattr(self.press, attr)
            if value != current:
                self.get_logger().warn(
                    f"Press retuned live: {attr} {current:g} -> {value:g}.")
                setattr(self.press, attr, value)

    def _on_wrench(self, msg):
        """TCP wrench -> press force, positive = pressing into the wall.

        ``force_torque_sensor_broadcaster`` reports in the tool frame, whose +Z
        is the axis the plate's sensors look along and the axis the wheel presses
        along. Pushing the wheel into the wall loads the sensor in -Z, so the
        sign flips here and the rest of the code only sees "how hard".

        Only the Z component is taken. The lateral components are real — the
        wheel drags along the wall as the sweep travels — but they are friction,
        not press, and folding them in would read a fast sweep as a hard press.
        """
        self.press_force = -float(msg.wrench.force.z)
        self.wrench_stamp = self._now()

    def _on_costmap(self, msg):
        self.costmap_msg = msg
        self.costmap_stamp = self._now()

    def _obstacle_field(self):
        """The cached distance field, rebuilt when a new costmap arrives.

        The costmap has its own frame (``odom`` for a rolling local costmap), and
        a grid cannot be rotated into another frame without resampling, so the
        field stays in that frame and the query points are brought TO it. The
        scan segment is transformed the same way before being masked out.
        """
        msg = self.costmap_msg
        if msg is None:
            return None
        if self._field_from is msg:
            return self.obstacle_field

        frame = msg.header.frame_id or self.map_frame
        segment = self._segment_in(frame)
        try:
            data = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)
            self.obstacle_field = ObstacleField.from_grid(
                data, msg.info.resolution,
                (msg.info.origin.position.x, msg.info.origin.position.y),
                self.avoid_config, mask_segment=segment)
        except Exception as exc:
            self.get_logger().warn(f"Unusable costmap: {exc}", throttle_duration_sec=10.0)
            self.obstacle_field = None
        self._field_from = msg
        return self.obstacle_field

    def _segment_in(self, frame):
        """The scan segment's endpoints in ``frame``, or None if TF cannot say.

        Masking the wrong strip is worse than masking none: the wall would keep
        pushing the base while some innocent patch of floor went unguarded. So a
        failed transform drops the mask rather than guessing.
        """
        if frame == self.map_frame:
            return (self.seg_start[:2], self.seg_end[:2])
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.map_frame, rclpy.time.Time())
        except Exception:
            self.get_logger().warn(
                f"No TF {self.map_frame}->{frame}; sweeping without the wall mask, "
                f"so the scanned wall itself may push the base away.",
                throttle_duration_sec=10.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        R = _quat_to_matrix(np.array([q.x, q.y, q.z, q.w]))[:2, :2]
        shift = np.array([t.x, t.y])
        return (R @ self.seg_start[:2] + shift, R @ self.seg_end[:2] + shift)

    def _avoidance_rows(self, n_arm):
        """Barrier rows for this cycle, or empty when there is nothing to avoid."""
        empty = (np.zeros((0, 3 + n_arm)), np.zeros(0))
        if not self.avoid:
            return empty
        field = self._obstacle_field()
        if field is None:
            self.get_logger().warn(
                "No costmap yet: sweeping with NO obstacle avoidance on the base.",
                throttle_duration_sec=5.0)
            return empty
        age = self._now() - (self.costmap_stamp or 0.0)
        if age > float(self.get_parameter("costmap_max_age").value):
            # STOP using it, don't just grumble. The local costmap is a rolling
            # window anchored to the robot: once it stops updating, the grid
            # describes where the robot USED to be, and the barrier starts
            # steering around obstacles that are no longer there while missing
            # anything that has appeared. Observed in Gazebo — a phantom
            # obstacle 0.07 m away pushed the base backwards and stalled the
            # last 6 cm of a sweep. Better to say plainly that there is no
            # avoidance than to act confidently on a stale picture.
            self.get_logger().error(
                f"Costmap is {age:.0f}s old — DROPPING base obstacle avoidance rather "
                f"than steering by a stale map. The sweep continues unguarded.",
                throttle_duration_sec=5.0)
            self.closest_obstacle = float("inf")
            return empty

        frame = self.costmap_msg.header.frame_id or self.map_frame
        pose = self._lookup(self.base_frame, source_frame=frame)
        if pose is None:
            self.get_logger().warn(
                f"No TF {frame}->{self.base_frame}; no obstacle rows this cycle.",
                throttle_duration_sec=5.0)
            return empty
        quat, position = pose
        yaw = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                         1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
        A, lower, _, closest = avoidance_rows(
            field, yaw, position, self.avoid_config, n_arm=n_arm)
        self.closest_obstacle = closest
        return A, lower

    # ------------------------------------------------------------------
    # Startup / teardown
    # ------------------------------------------------------------------
    def wait_until_ready(self, timeout_s=30.0):
        """Block (spinning) until model, joints, sensors and TF are all live.

        Everything the control law reads must be present BEFORE the first cycle:
        a whole-body command computed from a half-populated state would move the
        base with a stale arm pose. Returns True when ready.
        """
        if not self._wait_for_clock():
            self.get_logger().error(
                "use_sim_time is set but no /clock arrived; refusing to start a sweep "
                "whose timing would be meaningless."
            )
            return False
        deadline = self._now() + timeout_s
        missing = None
        while rclpy.ok() and self._now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            missing = self._missing_inputs()
            if not missing:
                # The posture task pulls back toward the pose the sweep starts
                # from — the one the FSM's pre-approach chose for this row.
                self.q_posture = self._arm_positions()
                if math.isnan(self.row_z):
                    self.row_z = float(self._tip_pose()[2, 3])
                    self.get_logger().info(
                        f"Holding the row height we start at: z={self.row_z:.3f} m.")
                return True
        self.get_logger().error(f"Not ready after {timeout_s:.0f}s; missing: {missing}.")
        return False

    def _wait_for_clock(self, timeout_s=15.0):
        """Wait for ROS time to become real before anything is timed against it.

        With ``use_sim_time`` the node's clock reads 0 until the first ``/clock``
        message lands, and then jumps to the simulator's time. A deadline
        computed in that window is instantly in the past — which is exactly how
        a healthy sweep reported "inputs never became available" a second after
        starting. This is the one place a wall clock is the right tool: we are
        waiting for the other clock to exist.
        """
        if not self.get_parameter("use_sim_time").value:
            return True
        end = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < end:
            if self._now() > 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._now() > 0.0

    def _missing_inputs(self):
        missing = []
        if self.chain is None:
            missing.append("robot_description")
        if self._arm_positions() is None:
            missing.append("joint_states")
        if self.distances is None:
            missing.append("distance_sensors")
        if self._mount_pose() is None:
            missing.append(f"TF {self.map_frame}->{self.arm_root_link}")
        if self._base_pose() is None:
            missing.append(f"TF {self.map_frame}->{self.base_frame}")
        if (self.avoid and self.costmap_msg is None
                and bool(self.get_parameter("avoid_require_costmap").value)):
            missing.append("costmap")
        return missing

    def start(self):
        """Begin the control loop.

        The first thing out of the door is the arm's CURRENT pose, not a
        motion: ``forward_position_controller`` has just taken over command
        interfaces the trajectory controller was holding, and until it hears
        from us it commands whatever value those interfaces already carry.
        Publishing the measurement first makes the handover a no-op instead of
        a jump. Harmless in velocity mode, where it is a zero.
        """
        self.arm_stream.initial_command(self._arm_positions())
        self.deadline = self._now() + self.timeout
        self.start_stamp = self._now()
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_step)
        self.get_logger().info(
            f"Sweeping at {self.control_rate:.0f} Hz (timeout {self.timeout:.0f}s)."
        )

    def halt(self):
        """Stop everything this node commands. Safe to call repeatedly.

        The arm's stop is NOT a zero — in position mode that would be a
        full-speed run to the zero configuration. ``ArmStream.hold`` knows what
        a stop means for the interface in use; nothing here should build one.
        """
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.control_timer = None
        self.cmd_vel_pub.publish(Twist())
        self.arm_stream.hold(self._arm_positions())
        self.u_prev = None
        self.command_stamp = None
        self.u_qp_prev = None
        self.qp_stamp = None
        self.holding_since = None

    def finish(self, status, reason=""):
        """End the sweep — but back the plate off the wall before saying so.

        The plate ends a sweep at its scanning standoff, centimetres off the
        surface, and everything that happens next needs it further out: the FSM
        retracts it before letting the base transit, and its planner has to solve
        that as a Cartesian goal through an octomap that contains the wall the
        plate is nearly touching. From there the search routinely fails ("no path
        found"), the FSM shrugs and carries on, and the base then transits with
        the arm still extended.

        This node is in a much better position to do it: it already holds the
        arm's velocity controller and it knows, from the plate's own sensors,
        which way "away from the wall" is. So retreat straight back along the
        sensed normal first, then hand the arm over — by which time the FSM's
        retraction has nothing left to do and never calls the planner at all.

        The terminal status is deliberately withheld until the retreat finishes:
        the FSM kills this process the moment it sees one.
        """
        self.pending_status = status if not reason else f"{status}: {reason}"
        level = self.get_logger().info if status == "succeeded" else self.get_logger().error
        level(f"Sweep {self.pending_status} (progress {self.progress:.2f}/"
              f"{self.segment_length:.2f} m).")

        target = float(self.get_parameter("retreat_standoff").value)
        if self.phase != "sweep" or target <= 0.0 or self.surface.distance is None:
            self._terminate()
            return
        self.cmd_vel_pub.publish(Twist())        # the base is done moving
        self.phase = "retreat"
        self.retreat_deadline = self._now() + float(
            self.get_parameter("retreat_timeout").value)
        self.get_logger().info(
            f"Retreating the plate from {self.surface.distance:.2f} m to {target:.2f} m "
            f"along the sensed normal before handing the arm back."
        )

    def _terminate(self):
        """Publish the withheld status and stop commanding anything."""
        self.phase = "done"
        self.halt()
        self.status = self.pending_status or "failed: ended without a status"
        self._publish_status()

    def _publish_status(self):
        self.status_pub.publish(String(data=self.status))

    # ------------------------------------------------------------------
    # State readers
    # ------------------------------------------------------------------
    def _arm_positions(self):
        try:
            return np.array([self.joint_positions[name] for name in self.arm_joints])
        except KeyError:
            return None

    def _turret_angle(self):
        """Turret joint angle (turret heading relative to the chassis).

        It decides how a commanded turret-frame twist lands on the chassis axes,
        so the base's actuator constraints are rebuilt around it every cycle.
        Falls back to 0 (chassis parked square, which is how a sweep starts).
        """
        return self.joint_positions.get(self.turret_joint, 0.0)

    def _lookup(self, target_frame, source_frame=None):
        try:
            tf = self.tf_buffer.lookup_transform(
                source_frame or self.map_frame, target_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def _mount_pose(self):
        """4x4 pose of the arm mount in the map frame.

        Straight from TF, so it carries the turret angle, the column's forward
        offset from the turret axis and the column's live height — none of which
        this node has to know about. Distinct from ``_base_pose``, which is the
        turret AXIS: the two are about 0.29 m apart horizontally.
        """
        result = self._lookup(self.arm_root_link)
        if result is None:
            return None
        quat, position = result
        T = np.eye(4)
        T[:3, :3] = _quat_to_matrix(quat)
        T[:3, 3] = position
        return T

    def _tip_pose(self):
        """4x4 pose of the plate tip in the map frame, or None if inputs are missing."""
        T_mount = self._mount_pose()
        q_arm = self._arm_positions()
        if T_mount is None or q_arm is None or self.chain is None:
            return None
        return T_mount @ self.chain.fk(q_arm)

    def _base_pose(self):
        """(yaw, xy) of the TURRET AXIS in the map frame.

        ``turret_footprint`` is the ground projection of ``turret_link``, i.e. of
        the axis the base rotates about and the point sim_controller's twist
        applies to — which is what the base Jacobian's lever arm must start from.
        """
        result = self._lookup(self.base_frame)
        if result is None:
            return None
        quat, position = result
        yaw = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                         1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
        return yaw, position

    # ------------------------------------------------------------------
    # The control loop
    # ------------------------------------------------------------------
    def _retreat_step(self):
        """Back the plate off along the sensed normal, arm only, base held still.

        Deliberately the same QP as the sweep with the base pinned to zero: the
        arm's joint limits and the plate's orientation task still apply, so the
        retreat cannot fling a joint into a stop or twist the plate on the way
        out. Any input it cannot trust ends the retreat rather than guessing a
        direction to move the arm in.
        """
        now = self._now()
        target = float(self.get_parameter("retreat_standoff").value)

        if now > self.retreat_deadline:
            self.get_logger().warn(
                f"Retreat timed out at {self.surface.distance:.2f} m (wanted "
                f"{target:.2f} m); returning the arm from here.")
            self._begin_return()
            return
        if self._stale_inputs(now) or not self.surface.update(self.distances):
            self.get_logger().warn("Lost the plate ranges mid-retreat; stopping here.")
            self._terminate()
            return

        T_mount = self._mount_pose()
        base = self._base_pose()
        q_arm = self._arm_positions()
        if T_mount is None or base is None or q_arm is None:
            self._terminate()
            return
        yaw, p_base = base
        J, T_plate = whole_body_jacobian(self.chain, q_arm, T_mount, yaw, p_base)
        R_plate = T_plate[:3, :3]
        m_hat = R_plate @ self.surface.normal_plate

        remaining = target - self.surface.distance
        if remaining <= 0.01:
            self.get_logger().info(
                f"Plate retreated to {self.surface.distance:.2f} m; arm is clear of "
                f"the wall.")
            self._begin_return()
            return

        p = self.get_parameter
        speed = min(float(p("retreat_speed").value), remaining)
        n_arm = self.chain.n_joints
        xdot = np.concatenate((-speed * m_hat, np.zeros(3)))
        weights = np.concatenate((np.full(3, float(p("weight_linear").value)),
                                  np.zeros(3)))
        damping = np.concatenate((np.array(p("damping_base").value, dtype=float),
                                  np.full(n_arm, float(p("damping_arm").value))))
        tasks = [Task(J, xdot, weights),
                 Task(np.diag(damping), np.zeros(3 + n_arm), 1.0)]

        lower_arm, upper_arm = self.chain.position_limits()
        arm_lo, arm_hi = joint_limit_bounds(
            q_arm, lower_arm, upper_arm, float(p("arm_qdot_max").value),
            margin=float(p("joint_limit_margin").value))
        # The retreat gets the same acceleration constraint as the sweep. It
        # starts with the wheel still loaded against the wall, so it is the last
        # place that should begin with a step.
        lo_all, hi_all = self._accel_bounds(
            np.concatenate((np.zeros(3), arm_lo)),      # base pinned: arm only
            np.concatenate((np.zeros(3), arm_hi)), now)
        solution = solve_velocity_qp(tasks, lo_all, hi_all)
        if not solution.ok:
            self.get_logger().warn("Retreat QP did not solve; stopping here.")
            self._terminate()
            return

        self._publish(solution.u, n_arm)
        self.get_logger().info(
            f"Retreating: d={self.surface.distance * 100:.1f}cm -> {target * 100:.0f}cm",
            throttle_duration_sec=1.0)

    def _begin_return(self):
        """Move to the joint-space return, or stop if there is nothing to return to."""
        if not self.return_joints or self.chain is None:
            self._terminate()
            return
        self.phase = "return"
        self.return_deadline = self._now() + float(
            self.get_parameter("return_timeout").value)
        self.get_logger().info(
            "Returning the arm to the configuration the planner left it in.")

    def _return_step(self):
        """Drive the arm joints back to ``return_joints``; the base stays put.

        Joint space on purpose. The target is a configuration the FSM's planner
        actually reached, so it is reachable, clear of the mast cylinder and
        something A* can plan out of — none of which any particular Cartesian
        standoff can guarantee.
        """
        now = self._now()
        q_arm = self._arm_positions()
        if q_arm is None or len(self.return_joints) != len(q_arm):
            self.get_logger().warn("No usable joint state for the return; stopping here.")
            self._terminate()
            return

        error = np.asarray(self.return_joints, dtype=float) - q_arm
        worst = float(np.max(np.abs(error)))
        if worst <= float(self.get_parameter("return_tolerance").value):
            self.get_logger().info(
                f"Arm back at the unfolded configuration (worst joint error "
                f"{worst:.3f} rad).")
            self._terminate()
            return
        if now > self.return_deadline:
            self.get_logger().warn(
                f"Return timed out with {worst:.2f} rad still to go; handing the arm "
                f"back as it is.")
            self._terminate()
            return

        limit = float(self.get_parameter("arm_qdot_max").value)
        qdot = np.clip(error * float(self.get_parameter("return_gain").value), -limit, limit)
        # ``error`` is in the CONTROLLER's joint order (both operands came from
        # arm_joints); _publish expects the chain's, so hand it over by name
        # rather than by position.
        by_name = dict(zip(self.arm_joints, qdot))
        qdot_chain = np.array([by_name[name] for name in self.chain.joint_names])
        self.cmd_vel_pub.publish(Twist())
        self._publish(np.concatenate((np.zeros(3), qdot_chain)), len(q_arm))
        self.get_logger().info(
            f"Returning: worst joint error {worst:.2f} rad", throttle_duration_sec=1.0)

    def _control_step(self):
        now = self._now()
        if self.phase == "done":
            return
        # Before anything else, and in every phase: is the robot in a state
        # where it will actually execute what it is told? A protective stop
        # freezes the arm while ros2_control carries on as if nothing happened —
        # the QP keeps solving, commands keep publishing, and the BASE keeps
        # driving along the wall with an arm that is no longer following it.
        # Nothing downstream would notice, so it has to be caught here.
        blocked = self.hardware.blocked()
        if blocked:
            self._strike(blocked)
            return
        if self.hardware.scaling() < self.min_speed_scaling:
            self._strike(f"execution speed scaled to {self.hardware.scaling():.2f}")
            return
        if self.phase == "retreat":
            self._retreat_step()
            return
        if self.phase == "return":
            self._return_step()
            return
        if self.status != "running":
            return
        if self.deadline is not None and now > self.deadline:
            self.finish("failed", f"timed out after {self.timeout:.0f}s")
            return

        stale = self._stale_inputs(now)
        if stale:
            self._strike(f"stale input: {stale}")
            return
        if not self.surface.update(self.distances):
            self._strike(f"only {self.surface.n_valid}/6 plate ranges valid — no plane fit")
            return

        T_mount = self._mount_pose()
        base = self._base_pose()
        q_arm = self._arm_positions()
        if T_mount is None or base is None or q_arm is None:
            self._strike("lost TF or joint states")
            return
        yaw, p_base = base
        # The turret joint angle relates the frame the twist is commanded in to
        # the chassis that has to yaw for it; both the speed cap and the actuator
        # rows below are built around it.
        phi = self._turret_angle()

        J, T_plate = whole_body_jacobian(self.chain, q_arm, T_mount, yaw, p_base)
        R_plate, p_plate = T_plate[:3, :3], T_plate[:3, 3]

        # --- Where are we, along the sensed surface? -------------------------
        direction = self.seg_end[:2] - self.seg_start[:2]
        length = np.linalg.norm(direction)
        if length < 1e-6:
            self.finish("failed", "degenerate segment (zero length)")
            return
        u_hat = np.array([direction[0] / length, direction[1] / length, 0.0])
        self.progress = float(np.dot(p_plate - np.array(
            [self.seg_start[0], self.seg_start[1], p_plate[2]]), u_hat))
        remaining = length - self.progress
        if remaining <= self.arrive_tolerance:
            self.finish("succeeded")
            return

        # Give up on a sweep that has stopped advancing, rather than sitting on
        # the wall until the full length/speed budget expires. Something is
        # holding the plate — an obstacle it is pressed against, a barrier it
        # cannot satisfy — and none of it improves by waiting several more
        # minutes with the arm loaded against the surface.
        if self.progress_stamp is None or self.progress > self.best_progress + 0.02:
            self.best_progress = self.progress
            self.progress_stamp = now
        else:
            stall = float(self.get_parameter("no_progress_timeout").value)
            if now - self.progress_stamp > stall:
                self.finish(
                    "failed",
                    f"no progress for {stall:.0f}s at {self.progress:.2f}/{length:.2f} m "
                    f"(something is holding the plate)")
                return

        # --- The sensed surface frame ---------------------------------------
        m_hat = R_plate @ self.surface.normal_plate      # plate -> wall, world axes
        t_hat = sweep_tangent(m_hat, u_hat)
        if t_hat is None:
            self._strike("scan direction is normal to the sensed surface")
            return
        distance = self.surface.distance
        if self.press is not None:
            # Pressing: the plate sits where the wall puts it, so a standoff
            # target is not a thing to hold and not a thing to police. The
            # protections here are the force limit and the min_distance
            # envelope, both inside AdmittancePress.
            self.standoff_strikes = 0
        elif abs(distance - self.standoff) > self.max_standoff_error:
            # A gap this far off is the plate meeting something the wall plane
            # does not explain (a pilaster, a fixture) or losing the wall
            # entirely — the case the old sweep could not see. Require several
            # cycles in a row so one bad plane fit cannot end a good sweep.
            self.standoff_strikes += 1
            if self.standoff_strikes >= self.standoff_error_cycles:
                self.finish(
                    "failed",
                    f"standoff {distance:.3f} m is {abs(distance - self.standoff):.3f} m off "
                    f"the {self.standoff:.2f} m target (obstacle or lost surface)")
                return
        else:
            self.standoff_strikes = 0

        # --- Task twist ------------------------------------------------------
        p = self.get_parameter
        if self.press is not None:
            # Force replaces distance on this axis, and only on this axis. The
            # press has its own clamp (press_v_max), sized for contact rather
            # than for closing a 20 cm gap, so v_normal_max does not apply.
            self._refresh_press_tuning()
            v_normal = self.press.update(self.press_force, distance)
            if self.press.fault:
                self.finish("failed", self.press.fault)
                return
            if self.press.stalled:
                self.finish(
                    "failed",
                    f"plate is at the {self.press.min_distance * 100:.1f} cm envelope "
                    f"with only {self.press.force:.1f} N on the wheel — the press is "
                    f"not reaching the wall (check the F/T tare and the plate offset)")
                return
        else:
            v_normal = _clamp(float(p("k_standoff").value) * (distance - self.standoff),
                              float(p("v_normal_max").value))
        v_height = _clamp(float(p("k_height").value) * (self.row_z - p_plate[2]),
                          float(p("v_normal_max").value))
        # Cap the reference by what the base can actually hold in THIS direction
        # (sideways is ~6x slower than straight ahead), then ease off over the
        # last few centimetres so the sweep stops ON the segment end instead of
        # overshooting past it between cycles. Asking for more than the base can
        # give does not make it go faster — sim_controller would rescale the
        # command, and the arm would try to take up the difference until it ran
        # out of reach.
        heading = math.atan2(t_hat[1], t_hat[0])
        reachable = self.limits.max_speed_along(
            heading - yaw, heading - (yaw - phi)) * float(p("sweep_speed_margin").value)
        speed = min(self.sweep_speed, remaining, reachable)
        if self.press is not None and not self.press.touched:
            # No travel until the wheel has reached the wall. Otherwise the base
            # sets off at sweep_speed while the arm is still closing the standoff
            # at press_seek_speed, and the first stretch of the segment is
            # crossed with the GPR scanning air.
            #
            # Gated on ``touched``, which LATCHES on the first contact, not on
            # ``in_contact``, which tracks the live contact state. That
            # distinction is the whole of this change: the press legitimately
            # drops back to SEEK over a hollow, a lip or a noisy reading, and
            # tying the base to it would stop and restart the base several times
            # a sweep — each restart a step from 0 to sweep_speed against a
            # loaded wheel, which scrubs it sideways instead of rolling it.
            # Once the wall has been found, the base sweeps to the end.
            #
            # Zero the tangent only. The normal, height and orientation tasks
            # keep running, which is what closes the gap in the first place.
            speed = 0.0
            if self.press_wait_since is None:
                self.press_wait_since = now
            # Standing still on purpose is not a stall. Without this the
            # no_progress_timeout would count the approach against the sweep and
            # kill it for doing exactly what it was told.
            self.best_progress = self.progress
            self.progress_stamp = now
            waited = now - self.press_wait_since
            if waited > float(p("press_contact_timeout").value):
                self.finish(
                    "failed",
                    f"the press never reached the wall: {waited:.0f}s seeking at "
                    f"{distance * 100:.1f} cm with {self.press.force:+.1f} N on the "
                    f"wheel against a {self.press.target_force:.0f} N target. "
                    f"Sweeping without contact would record air.")
                return
        v_ref = speed * t_hat + v_normal * m_hat + v_height * np.array([0.0, 0.0, 1.0])

        R_target = plate_orientation_target(m_hat)
        if R_target is None:
            self._strike("sensed surface normal is vertical")
            return
        w_ref = rotation_error(R_plate, R_target) * float(p("k_align").value)
        w_max = float(p("w_align_max").value)
        if np.linalg.norm(w_ref) > w_max:
            w_ref = w_ref * (w_max / np.linalg.norm(w_ref))
        xdot = np.concatenate((v_ref, w_ref))

        # --- Resolve it across the 9 DOF -------------------------------------
        n_arm = self.chain.n_joints
        weights = np.concatenate((np.full(3, float(p("weight_linear").value)),
                                  np.full(3, float(p("weight_angular").value))))
        damping = np.concatenate((np.array(p("damping_base").value, dtype=float),
                                  np.full(n_arm, float(p("damping_arm").value))))
        # The BASE owns the tangent, the ARM owns the normal — the task-frame
        # ownership the design calls for. Without this the damping weights decide
        # it instead, and they say the base is ~100x cheaper than the arm on
        # EVERY axis, so the base ends up carrying the standoff corrections too:
        # it creeps toward or away from the wall for the whole sweep while the
        # arm barely moves. A steady 1 cm of standoff error is then enough to
        # walk the base out of position over a few minutes, until it sits inside
        # Nav2's collision radius and cannot transit to the next segment.
        # Penalising only the base's velocity ALONG the sensed normal costs the
        # sweep nothing, since the sweep runs along the tangent, which is
        # perpendicular to it.
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        base_normal_row = np.zeros((1, 3 + n_arm))
        base_normal_row[0, :2] = m_hat[:2] @ rotation

        J_task, xdot_task = J, xdot
        press_task = []
        if self.press is not None:
            # A press reference is TINY — tens of microns per second, three or
            # four orders below the sweep travel. Left inside the pooled linear
            # task it is simply lost: measured here, a 2.5e-4 m/s press command
            # sat 26x below the solve's own residual, and the plate drifted off
            # the wall while the loop politely asked it not to. Weighting the
            # whole linear task harder does not help either, since that scales
            # the tangent and the height with it.
            #
            # So separate the axes. The pooled task is projected onto the tangent
            # plane and keeps the travel, the height and the orientation; the
            # normal gets one row of its own at a weight that makes it effectively
            # authoritative. This is the same move as pinning the base travel: the
            # quantity that has to be exact stops being one voice in a sum.
            #
            # A weight rather than a hard constraint, deliberately. An equality
            # row here could conflict with a joint limit and make the whole solve
            # infeasible, which would stop the robot dead mid-press; a large
            # weight degrades instead, and the force loop's own clamps bound what
            # it can ask for anyway.
            projector = np.eye(3) - np.outer(m_hat, m_hat)
            J_task = J.copy()
            J_task[:3, :] = projector @ J[:3, :]
            xdot_task = np.concatenate((projector @ v_ref, w_ref))
            press_task = [Task(np.atleast_2d(m_hat @ J[:3, :]),
                               np.array([v_normal]),
                               float(p("weight_press_normal").value))]

        tasks = press_task + [
            Task(J_task, xdot_task, weights),
            Task(np.diag(damping), np.zeros(3 + n_arm), 1.0),
            Task(base_normal_row, np.zeros(1), float(p("weight_base_normal").value)),
            # Posture: pull the arm back toward the pose the sweep started from,
            # so the redundancy is spent keeping the arm workable instead of
            # slowly stretching it out over the length of the wall.
            Task(np.hstack((np.zeros((n_arm, 3)), np.eye(n_arm))),
                 _clamp_norm(float(p("k_posture").value) * (self.q_posture - q_arm),
                             float(p("posture_rate_max").value)),
                 float(p("weight_posture").value)),
        ]
        # Carry on from last cycle unless the task gives a reason not to. See
        # weight_smoothness. Only once there IS a last cycle: at the start of a
        # sweep there is nothing to be continuous with, and seeding it with
        # zeros would ask the first solve to stay stopped.
        smoothness = float(p("weight_smoothness").value)
        if self.u_qp_prev is not None and smoothness > 0.0:
            tasks.append(Task(np.eye(3 + n_arm), self.u_qp_prev, smoothness))

        lower_arm, upper_arm = self.chain.position_limits()
        arm_lo, arm_hi = joint_limit_bounds(
            q_arm, lower_arm, upper_arm, float(p("arm_qdot_max").value),
            margin=float(p("joint_limit_margin").value))
        base_lo, base_hi = box_bounds(self.limits)
        # Acceleration as a CONSTRAINT, before the solve. Narrowing the box here
        # rather than clipping the answer afterwards is what makes the solution
        # both smooth and feasible, and it lands in exactly the right place for
        # the base travel pin below: the pin already clips itself into
        # [base_lo[0], base_hi[0]], so a pin that has just jumped from zero to
        # sweep_speed now becomes a short ramp instead of a step, at the one
        # moment the wheel is newly loaded against the wall.
        lo_all, hi_all = self._accel_bounds(
            np.concatenate((base_lo, arm_lo)),
            np.concatenate((base_hi, arm_hi)), now)
        base_lo, base_hi = lo_all[:3], hi_all[:3]
        arm_lo, arm_hi = lo_all[3:], hi_all[3:]
        A_ineq, ineq_lo, ineq_hi = constraint_rows(self.limits, phi)
        A_ineq = np.hstack((A_ineq, np.zeros((A_ineq.shape[0], n_arm))))

        # Obstacle barriers go in SOFT: a base pinned inside its margin can be
        # asked to retreat faster than it is able to strafe, and a hard row there
        # would make the whole solve infeasible — stopping the robot instead of
        # backing it off. Actuator limits stay hard.
        A_avoid, avoid_lo = self._avoidance_rows(n_arm)

        # The scanned wall is masked out of the costmap barrier by design, which
        # leaves nothing at all watching the one obstacle the base is guaranteed
        # to be near. Close that with a barrier against the SENSED wall instead:
        # the plate measures where the surface is, so the base's distance to it
        # is (plate standoff + how far the base sits behind the plate) along the
        # normal. Keep that above a floor, or the base ends up closer to the wall
        # than Nav2's own footprint allows and the next transit cannot plan.
        base_gap = float(np.dot(p_plate[:2] - p_base[:2], m_hat[:2])) + distance
        floor = float(p("base_wall_min_gap").value)
        if floor > 0.0:
            A_avoid = np.vstack((A_avoid, -base_normal_row))
            avoid_lo = np.concatenate((
                avoid_lo, [-float(p("avoid_alpha").value) * (base_gap - floor)]))
            if base_gap < floor:
                self.get_logger().warn(
                    f"Base is {base_gap:.2f} m from the wall, inside the {floor:.2f} m "
                    f"floor; backing it off before it gets stuck against the surface.",
                    throttle_duration_sec=5.0)

        # --- Pin the base's travel, so the wheels do one steady thing ---------
        # Collapsing the forward bound to a point makes the base's along-wall
        # speed an input to the solve rather than an output of it. The remaining
        # 8 DOF then resolve everything else around a base that is no longer
        # free to hesitate. ``speed`` already carries the ease-off over the last
        # few centimetres, so this tracks the reference rather than being a
        # literal constant that would overshoot the segment end.
        #
        # Only ONE case hands the travel back to the solver outright, because
        # only one makes pinning actively wrong rather than merely inconvenient.
        # An obstacle narrows the bound instead of removing it — see below.
        self.base_travel_pinned = False
        self.base_travel_capped = False
        if self.base_constant_travel:
            # Sweep direction expressed in the turret frame, where the twist is
            # commanded. During a wall scan this is ~[±1, 0]: the turret faces
            # along the wall, so its forward axis IS the travel axis.
            t_base = rotation.T @ t_hat[:2]
            if abs(float(t_base[0])) < self.base_travel_min_alignment:
                # The forward axis is not the travel axis. Pinning it would hold
                # the base at some fraction of the sweep speed along the wrong
                # direction and leave the arm to make up an ever-growing rest.
                self.get_logger().warn(
                    f"Turret is {math.degrees(math.acos(min(1.0, abs(float(t_base[0]))))):.0f} deg "
                    f"off the sweep direction: letting the solver apportion the base "
                    f"travel rather than pinning an axis that is not the travel axis.",
                    throttle_duration_sec=5.0)
            else:
                pin = float(np.clip(speed * float(t_base[0]), base_lo[0], base_hi[0]))
                engaged = (self.closest_obstacle < float(p("avoid_influence").value)
                           or base_gap < floor)
                if engaged:
                    # The obstacle rows go in SOFT so they can never make the
                    # solve infeasible, and a two-sided pin is HARD, so the pin
                    # would win against them and carry the base on into whatever
                    # they were avoiding. The barrier has to be able to act.
                    #
                    # But it only ever needs to act in ONE direction. Slowing the
                    # base, or stopping it, is the whole of what a barrier is
                    # for; nothing about avoiding an obstacle is served by
                    # running the base FASTER than the scan speed or reversing
                    # it. Releasing the bound entirely granted both, and that is
                    # what the base actually did — measured over one Gazebo
                    # segment, 0.002 to 0.063 m/s against a 0.030 reference,
                    # while a real corner sat inside the influence radius for the
                    # first quarter of the sweep.
                    #
                    # So keep the half of the bound that costs the barrier
                    # nothing. Zero stays reachable, full authority to slow is
                    # preserved, and the scan speed remains a ceiling.
                    base_lo[0], base_hi[0] = min(0.0, pin), max(0.0, pin)
                    self.base_travel_capped = True
                else:
                    base_lo[0] = base_hi[0] = pin
                    self.base_travel_pinned = True

        solve_started = time.monotonic()
        solution = solve_velocity_qp(
            tasks,
            np.concatenate((base_lo, arm_lo)), np.concatenate((base_hi, arm_hi)),
            A_ineq=A_ineq, ineq_lo=ineq_lo, ineq_hi=ineq_hi,
            soft_ineq=A_avoid if A_avoid.shape[0] else None, soft_lo=avoid_lo,
            soft_weight=float(p("avoid_slack_weight").value))
        self.solve_seconds = time.monotonic() - solve_started
        if not solution.ok:
            self._strike(f"QP did not solve ({solution.status})")
            return
        if solution.slack > 1e-3:
            self.get_logger().warn(
                f"Obstacle {self.closest_obstacle:.2f} m away: retreating as fast as "
                f"the base allows, still {solution.slack:.3f} m/s short of the barrier.",
                throttle_duration_sec=2.0)
        if solution.solver.endswith("(box-only)"):
            self.get_logger().warn(
                "OSQP unavailable: solving with a box-only fallback, so the base's "
                "chassis/turret/wheel limits are NOT enforced by the solver.",
                throttle_duration_sec=10.0)

        self.holding_since = None
        self._publish(solution.u, n_arm)
        self._log_cycle(solution, distance, remaining, phi)

    def _accel_bounds(self, lo, hi, now):
        """Narrow the QP's box bounds to what the acceleration limit allows.

        The bound belongs HERE, in the constraints, and not only in
        ``slew_limit`` after the solve. Clipping the answer afterwards changes
        it into one the solver never checked: the base travel pin, the obstacle
        barriers and the joint limits were all satisfied by the number that came
        out, and the clipped number satisfies none of them by construction. It
        is also why the arm steps — the QP is memoryless, its active set moves
        between cycles, and a post-clip does not make the underlying solution
        any smoother, it just truncates the jump.

        Given to the solver instead, smoothness and feasibility are traded off
        against each other in one place, by the thing that knows both.

        ``slew_limit`` stays where it is. It is no longer the primary bound but
        it is still the only thing watching the PUBLISHED command, which carries
        the speed scaling: a teach-pendant slider moving under our feet steps
        that command without the QP ever seeing it.
        """
        if self.u_qp_prev is None:
            return lo, hi
        # The MEASURED interval since the last solve, clamped. Nominal would be
        # wrong in the direction that matters here: the loop has been seen at
        # 20 Hz against a nominal 50, and bounding a 50 ms step by 20 ms of
        # acceleration would hold the robot to 40% of the acceleration it is
        # configured for. The upper clamp keeps one late cycle after a stall
        # from authorising a jump.
        nominal = 1.0 / self.control_rate
        elapsed = (now - self.qp_stamp) if self.qp_stamp else nominal
        dt = float(min(max(elapsed, nominal), 3.0 * nominal))
        step = np.abs(self.accel_max) * dt
        lo_new = np.maximum(lo, self.u_qp_prev - step)
        hi_new = np.minimum(hi, self.u_qp_prev + step)
        # The two can cross, and an empty box is an infeasible solve — the robot
        # stopping dead, which is the one outcome worse than a step. It happens
        # whenever the previous solution is already outside the new box: the
        # joint-limit bounds move as the arm does, and a caller may pin a DOF
        # (the retreat pins the base to zero) that was moving last cycle. Where
        # they cross, hand back the box unnarrowed and let slew_limit take it.
        crossed = lo_new > hi_new
        if np.any(crossed):
            lo_new = np.where(crossed, lo, lo_new)
            hi_new = np.where(crossed, hi, hi_new)
        return lo_new, hi_new

    def _stale_inputs(self, now):
        stale = []
        if self.joint_stamp is None or now - self.joint_stamp > self.max_data_age:
            stale.append("joint_states")
        if self.distance_stamp is None or now - self.distance_stamp > self.max_data_age:
            stale.append("distance_sensors")
        if self.press is not None and (
                self.wrench_stamp is None or now - self.wrench_stamp > self.max_data_age):
            # A press with no force feedback is an arm driving at a wall on a
            # timer. Hold, exactly as for a lost surface.
            stale.append("wrench")
        return ", ".join(stale)

    def _strike(self, reason):
        """One unusable cycle: stop the robot, and give up if it keeps happening.

        Stopping (rather than letting the last command stand) matters — every
        consumer here would happily keep driving on it. What "stop" means
        differs by interface, so the arm's is delegated to ``ArmStream.hold``,
        which in position mode re-seeds at the measurement: the arm stays where
        it is and the integrator resumes from the truth rather than from a
        setpoint that has been sitting ahead of a stationary robot.
        """
        now = self._now()
        if self.holding_since is None:
            self.holding_since = now
        self.cmd_vel_pub.publish(Twist())
        self.arm_stream.hold(self._arm_positions())
        # Zero, not None: the robot has just been commanded to a stop, so that
        # IS the current command and the cycle that resumes the sweep must ramp
        # up from it. Clearing the history instead would let the loop jump
        # straight back to full sweep speed the moment the input recovers —
        # exactly the step the acceleration bound exists to prevent, at the one
        # moment the robot's state is least well understood.
        self.u_prev = np.zeros_like(self.accel_max)
        self.command_stamp = now
        # Same reasoning for the solver's own reference: the robot has just been
        # stopped, so zero IS where the next solve has to accelerate from.
        self.u_qp_prev = np.zeros_like(self.accel_max)
        self.qp_stamp = now
        held = now - self.holding_since
        if held >= self.max_hold_seconds:
            self.finish("failed", f"{reason} (held {held:.1f}s)")
        else:
            self.get_logger().warn(f"Holding: {reason}.", throttle_duration_sec=1.0)

    def _publish(self, u, n_arm):
        """The single funnel every command leaves through.

        Three things happen here that the QP knows nothing about, in this order:

        1. **Speed scaling.** The UR executes the arm at a fraction set by the
           teach pendant and the safety configuration; the base has never heard
           of it. Scaling the WHOLE vector keeps base and arm on the same clock
           — the alternative is a base sweeping at full speed alongside an arm
           that cannot keep up with the standoff corrections meant to accompany
           it. Reads 1.0 wherever nothing publishes it, so this is inert in sim.
        2. **Acceleration bound.** The QP is memoryless and its solution steps
           when the active set changes; ``speedj`` is given 40 rad/s^2 to make
           that step with.
        3. **Split and stream.** Base Twist in the turret frame (as
           sim_controller's ``cmd_type: relative`` expects), arm through
           ``ArmStream``, which decides whether the wire carries velocities or
           integrated setpoints.
        """
        # The solver's own answer, kept before anything is done to it. This is
        # what the acceleration bound in _accel_bounds measures the NEXT solve
        # against, and it is recorded here rather than at the one call site that
        # first needed it: the retreat and the return publish through this
        # method too, and a reference that only some phases update would leave
        # the retreat bounded against a stale sweep velocity.
        u_qp = np.asarray(u, dtype=float)
        u = u_qp * self.hardware.scaling()

        # The NOMINAL control period, not the measured one. Integrating the
        # elapsed time looks more faithful and is worse in both directions that
        # matter: a burst of early cycles under-integrates the setpoint (the arm
        # silently runs slow), and a cycle that arrives late after a stall
        # advances the setpoint by the whole gap at once — a jump, at the one
        # moment the robot is least understood. With a fixed period a late loop
        # simply moves the arm slower, which is the safe direction, and the
        # divergence is reported below rather than absorbed.
        dt = 1.0 / self.control_rate
        u = slew_limit(u, self.u_prev, self.accel_max, dt)
        self.u_prev = u

        now = self._now()
        # Captured before command_stamp is overwritten below: the
        # diagnostics need the gap between cycles, and reading it after the
        # update would record zero every time.
        self.cycle_period = (now - self.command_stamp) if self.command_stamp else 0.0
        if self.command_stamp is not None:
            measured = now - self.command_stamp
            if measured > 2.0 * dt:
                # The setpoint stream is thinner than the arm was promised, so
                # it will track slower than commanded. Usually CPU starvation.
                self.get_logger().warn(
                    f"Control loop is running at {1.0 / max(measured, 1e-6):.0f} Hz, "
                    f"not the {self.control_rate:.0f} Hz it commands for; the arm "
                    f"will move slower than the base.",
                    throttle_duration_sec=5.0)
        self.command_stamp = now

        twist = Twist()
        twist.linear.x, twist.linear.y = float(u[0]), float(u[1])
        twist.angular.z = float(u[2])
        self.cmd_vel_pub.publish(twist)

        # Reorder from the CHAIN's joints to the CONTROLLER's, which is what
        # goes on the wire.
        by_name = dict(zip(self.chain.joint_names, u[3:3 + n_arm]))
        qdot = np.array([float(by_name.get(name, 0.0)) for name in self.arm_joints])
        self.arm_stream.send(qdot, dt, self._arm_positions())

        self._publish_diagnostics(now, u_qp, u)
        self.u_qp_prev = u_qp
        self.qp_stamp = now

    # Layout of the diagnostics array. Published as one flat Float64MultiArray
    # so it needs no message package of its own; the cost of that is that the
    # layout lives here and nowhere else, so keep this in step with
    # _publish_diagnostics and treat it as the format's documentation.
    #
    #   [0]        wall-clock seconds since the sweep's first command
    #   [1]        measured control period, s      <- jitter shows up here
    #   [2]        QP solve duration, s
    #   [3]        robot speed scaling, 0..1
    #   [4]        press state: -1 none, 0 tare, 1 seek, 2 press
    #   [5]        plate distance to the sensed wall, m
    #   [6]        press force, N, filtered and de-biased
    #   [7]        setpoint lead over the measured arm, rad
    #   [8:8+n]    what the QP asked for       (base 3, then arm)
    #   [8+n:8+2n] what was published          (after scaling and slew)
    #   [8+2n:...] what the arm actually did   (arm joints only, from
    #                                           /joint_states)
    DIAG_HEADER = 8

    def _publish_diagnostics(self, now, u_qp, u_published):
        """One row per control cycle, for the plot that localises a jerky arm.

        Deliberately records all three velocities rather than just the last one.
        Watching the robot cannot distinguish a control law that is producing
        steps from a smooth law whose commands are arriving too slowly, and
        those two faults have opposite fixes.
        """
        if self.diag_pub is None:
            return
        n = 3 + self.chain.n_joints
        if self.press is None:
            state = -1.0
            force = 0.0
        else:
            state = float({TARE: 0, SEEK: 1}.get(self.press.state, 2))
            force = self.press.force
        measured = [self.joint_velocities.get(name, float("nan"))
                    for name in self.arm_joints]
        row = [
            now - self.start_stamp if self.start_stamp else 0.0,
            self.cycle_period,
            self.solve_seconds,
            self.hardware.scaling(),
            state,
            self.surface.distance if self.surface.distance is not None else float("nan"),
            force,
            self.arm_stream.lead(self._arm_positions()),
        ]
        row.extend(float(v) for v in np.asarray(u_qp, dtype=float)[:n])
        row.extend(float(v) for v in np.asarray(u_published, dtype=float)[:n])
        row.extend(measured)
        self.diag_pub.publish(Float64MultiArray(data=row))

    def _log_cycle(self, solution, distance, remaining, phi):
        w_left, w_right, phi_dot, w_chassis = wheel_and_turret_rates(
            self.limits, solution.u[:3], phi)
        clearance = ("--" if not np.isfinite(self.closest_obstacle)
                     else f"{self.closest_obstacle:.2f}m")
        # The setpoint's lead over the measured joints is the tell for an arm
        # that is not keeping up: it sits near zero when all is well and pins at
        # arm_stream_max_lead under speed scaling or a stop.
        lead = self.arm_stream.lead(self._arm_positions())
        press = ("" if self.press is None else
                 f"press={self.press.force:+.1f}N/{self.press.target_force:.0f} "
                 f"[{self.press.state.upper() if self.press.state != SEEK else 'seek'}] "
                 f"bias={self.press.bias:+.1f}N | ")
        self.get_logger().info(
            f"{press}d={distance * 100:.1f}cm tilt={math.degrees(self.surface.tilt()):.1f}deg "
            f"left={remaining:.2f}m | base{'*' if self.base_travel_pinned else ''}"
            f"{'~' if self.base_travel_capped else ''}="
            f"({solution.u[0]:+.3f}, {solution.u[1]:+.3f}, "
            f"{solution.u[2]:+.3f}) w_chassis={w_chassis:+.2f} phi_dot={phi_dot:+.2f} "
            f"| arm max={np.max(np.abs(solution.u[3:])):.3f} rad/s lead={lead:.3f}rad "
            f"| {self.hardware.describe()} "
            f"| clear={clearance} | residual={solution.task_residual:.4f}",
            throttle_duration_sec=1.0)


def _clamp(value, limit):
    return float(max(-limit, min(limit, value)))


def _clamp_norm(vector, limit):
    """Scale ``vector`` down to ``limit`` in norm, preserving its direction."""
    norm = float(np.linalg.norm(vector))
    if limit <= 0.0 or norm <= limit or norm < 1e-12:
        return vector
    return vector * (limit / norm)


def _quat_to_matrix(q):
    """Rotation matrix from (x, y, z, w) — avoids a scipy import in the loop."""
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main(args=None):
    from rclpy.signals import SignalHandlerOptions

    from .controller_switch import ArmControllerSwitch

    # Handle the stop signal ourselves. rclpy's default handler invalidates the
    # context the moment SIGINT lands, and this node has real work to do on the
    # way out: stop the base and arm, and hand the arm's command interfaces back
    # to the trajectory controller. With the default handler both of those throw
    # ("publisher's context is invalid") and the arm is left parked on a
    # streaming controller nobody is feeding — every later FSM arm goal would
    # then silently do nothing. SIGTERM is treated the same way, so a supervisor
    # that stops the node also gets a clean handover.
    #
    # A SIGKILL or a hard crash still skips all of it, which is the case the
    # position interface is chosen for: the last setpoint the controller holds
    # is a POSE, so an abandoned arm stops rather than continues. The FSM's
    # _ensure_arm_trajectory_controller then puts the controller back.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WholeBodySweepNode()
    switch = ArmControllerSwitch(node, controller=node.arm_controller)

    stopping = False

    def _request_stop(signum, _frame):
        nonlocal stopping
        stopping = True
        # A second signal should still be able to kill us outright.
        signal.signal(signum, signal.SIG_DFL)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        if not node.wait_until_ready():
            node.finish("failed", "inputs never became available")
        elif not switch.claim():
            node.finish("failed",
                        f"could not activate '{node.arm_controller}' for the arm")
        else:
            # start() publishes the arm's current pose immediately, closing the
            # window between the switch landing and the first real command.
            node.start()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt()
        switch.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
