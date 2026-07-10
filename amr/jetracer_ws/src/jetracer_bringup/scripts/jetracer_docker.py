#!/usr/bin/env python3
"""Named-dock Ackermann docking via Pure Pursuit visual servoing, with undock.

Each dock id (dock0/1/2) maps to an AprilTag frame (dock_0/1/2) and a surveyed
map position. A trigger selects which dock to approach; the node looks that tag
up in TF directly, so "dock0" and "dock1" really go to different tags (unlike
the old version, which homed on the nearest visible tag).

Phase 1 — Staging (Nav2):
  Using the selected dock's surveyed map position, compute a staging pose
  `staging_distance` short of it on the robot->dock line and send NavigateToPose
  so Nav2 drives the car across the map to in front of the dock -- even when the
  tag is not yet visible. Skipped if skip_staging, or if the dock has no map
  pose and the tag is not visible, or if Nav2/TF is unavailable.

Phase 2 — Approach (Pure Pursuit):
  Steer toward the selected tag (in base_footprint) with the bicycle-model Pure
  Pursuit law; steer_gain shortens the effective lookahead so the dock is
  centered sooner. Stop when within docking_threshold.

Search pivot:
  If the tag is not visible when the approach starts (e.g. just arrived at the
  staging pose and it is out of the camera FOV), rotate in place with a
  multi-point (K-turn) maneuver toward the dock bearing until the tag is seen,
  then resume the approach.

Undock:
  Reverse straight back to ~the staging pose (the dock's staging distance), then
  idle. Falls back to undock_distance if the dock has no staging distance.

Sequence:
  /dock_sequence runs several docks in turn ("dock0,dock2"; append ",loop" to
  cycle). Between docks it undocks to ~staging ONLY if it actually docked; if a
  dock attempt failed (no tag / pivot timeout) it drives straight to the next.

State:
  Publishes the current phase as std_msgs/String on /docking_state (latched +
  5 Hz heartbeat) so a sequencer can wait on transitions.

Triggers:
    ros2 topic pub --once /dock_robot    std_msgs/msg/String '{data: "dock1"}'
    ros2 topic pub --once /dock_sequence std_msgs/msg/String '{data: "dock0,dock2,loop"}'
    ros2 topic pub --once /undock_robot  std_msgs/msg/Bool   '{data: true}'
    ros2 topic pub --once /abort_docking std_msgs/msg/Bool   '{data: true}'
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class JetRacerDocker(Node):
    def __init__(self):
        super().__init__('jetracer_docker')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheelbase',          0.165)
        self.declare_parameter('delta_max_deg',      30.0)
        self.declare_parameter('approach_speed',     0.10)
        self.declare_parameter('docking_threshold',  0.15)
        self.declare_parameter('detection_timeout',  0.5)
        self.declare_parameter('staging_distance',   1.5)
        self.declare_parameter('skip_staging',       False)
        self.declare_parameter('steer_gain',         1.0)
        # If the tag is lost while already within this distance (it has left the
        # camera FOV on close approach), finish the last stretch open-loop on
        # odometry instead of stalling on stale detection. Lose it farther out
        # and it is treated as a real loss (keep waiting, don't false-dock).
        self.declare_parameter('blind_dock_distance', 0.45)
        # Safety guard: if the tag is lost mid-approach while still too far for the
        # blind creep (a real loss, not just leaving the FOV), the car holds and
        # waits for re-detection. Cap that wait — after this many seconds with no
        # re-detection, abort the attempt (phase -> idle) instead of waiting
        # forever. In a sequence this lets the run move on to the next dock.
        # <=0 disables the timeout (wait indefinitely, old behaviour).
        self.declare_parameter('approach_lost_timeout', 15.0)
        # Between docks in a sequence the car first reverses off the dock it just
        # left (undock), then SETTLES before driving to the next one: it clears
        # the Nav2 costmaps (so the just-left dock is re-seeded from a fresh scan,
        # not stale inflation) and pauses this long. Prevents replanning forward
        # into the dock it is still sitting in front of.
        self.declare_parameter('interdock_settle_time', 2.0)
        # Search pivot: if the tag is not seen at the staging pose, rotate in place
        # with a multi-point (K-turn) maneuver toward the dock bearing until the tag
        # enters the camera FOV. pivot_segment_dist = length of each forward/reverse
        # leg; pivot_align_tolerance = heading error at which the robot counts as
        # facing the dock (stop searching if still no tag); pivot_timeout caps it.
        self.declare_parameter('pivot_speed',           0.08)
        self.declare_parameter('pivot_segment_dist',    0.35)
        self.declare_parameter('pivot_align_tolerance', 0.15)
        self.declare_parameter('pivot_timeout',         25.0)
        # Dock id -> AprilTag frame and surveyed map position (parallel lists).
        # A dock with map pose (0, 0) is treated as "unsurveyed": staging then
        # falls back to the tag's TF (requires the tag to be visible).
        self.declare_parameter('dock_ids',        ['dock0', 'dock1', 'dock2'])
        self.declare_parameter('dock_tag_frames', ['dock_0', 'dock_1', 'dock_2'])
        self.declare_parameter('dock_pose_x',     [0.0, 0.0, 0.0])
        self.declare_parameter('dock_pose_y',     [0.0, 0.0, 0.0])
        # Robot heading (map yaw) when docked, per dock. The staging pose is placed
        # staging_distance in front of the dock along this axis, facing in. NaN =
        # unknown -> fall back to facing the dock from the robot's current position.
        self.declare_parameter('dock_pose_yaw',
                               [float('nan'), float('nan'), float('nan')])
        # Per-dock staging distance (m), parallel to dock_ids. <=0 -> use the scalar
        # staging_distance fallback for that dock.
        self.declare_parameter('dock_staging_dist',   [0.0, 0.0, 0.0])
        # Undock: by default reverse back to ~the staging pose (the dock's staging
        # distance); falls back to undock_distance if that dock has none.
        self.declare_parameter('undock_distance',     0.5)
        self.declare_parameter('undock_speed',        0.10)
        # Docking sequence: seconds to dwell while docked before moving to the next.
        self.declare_parameter('sequence_dwell',      2.0)
        # Scale on the undock reverse distance (2.0 = back out ~twice as far as the
        # staging pose, for extra clearance before maneuvering to the next dock).
        self.declare_parameter('undock_back_scale',   2.0)
        self.declare_parameter('base_frame',  'base_footprint')
        self.declare_parameter('fixed_frame', 'odom')
        # Planner switching: which loaded Nav2 planner the BT should use for the
        # staging navigation (tight approach) vs. everything else. We publish the id
        # to planner_selector_topic; the BT's PlannerSelector picks it up per replan.
        self.declare_parameter('staging_planner_id',     'SmacHybrid')
        self.declare_parameter('transit_planner_id',     'GridBased')
        self.declare_parameter('planner_selector_topic', 'planner_selector')
        # Distance (m) from the staging pose at which we hand off from the transit
        # planner (GridBased/NavFn: smooth, forward-only) to the staging planner
        # (SmacHybrid: heading-aware, can reverse to park). The long approach runs
        # on GridBased so it isn't jerky; only this final stretch runs on SmacHybrid
        # so the reversing/heading lineup into the dock still works. 0 -> switch
        # to the staging planner immediately (old behaviour).
        self.declare_parameter('last_mile_dist',         1.5)

        self.L          = self.get_parameter('wheelbase').value
        self.delta_max  = math.radians(self.get_parameter('delta_max_deg').value)
        self.speed      = self.get_parameter('approach_speed').value
        self.dock_thr   = self.get_parameter('docking_threshold').value
        self.det_tmo    = self.get_parameter('detection_timeout').value
        self.stg_dist   = self.get_parameter('staging_distance').value
        self.skip_stg   = self.get_parameter('skip_staging').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.blind_dist = self.get_parameter('blind_dock_distance').value
        self.approach_lost_tmo = self.get_parameter('approach_lost_timeout').value
        self.settle_time = self.get_parameter('interdock_settle_time').value
        self.pivot_speed     = self.get_parameter('pivot_speed').value
        self.pivot_seg_dist  = self.get_parameter('pivot_segment_dist').value
        self.pivot_align_tol = self.get_parameter('pivot_align_tolerance').value
        self.pivot_timeout   = self.get_parameter('pivot_timeout').value
        self.undock_dist  = self.get_parameter('undock_distance').value
        self.undock_speed = self.get_parameter('undock_speed').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.fixed_frame  = self.get_parameter('fixed_frame').value
        self.staging_planner = self.get_parameter('staging_planner_id').value
        self.transit_planner = self.get_parameter('transit_planner_id').value
        self.last_mile_dist  = self.get_parameter('last_mile_dist').value
        planner_sel_topic    = self.get_parameter('planner_selector_topic').value

        ids    = list(self.get_parameter('dock_ids').value)
        frames = list(self.get_parameter('dock_tag_frames').value)
        xs     = list(self.get_parameter('dock_pose_x').value)
        ys     = list(self.get_parameter('dock_pose_y').value)
        yaws   = list(self.get_parameter('dock_pose_yaw').value)
        self.dock_to_tag = dict(zip(ids, frames))
        self.dock_xy     = {i: (x, y) for i, x, y in zip(ids, xs, ys)}
        # None = heading unknown (NaN in params) -> fall back to robot->dock line.
        self.dock_yaw    = {i: (None if math.isnan(y) else y)
                            for i, y in zip(ids, yaws)}
        # Per-dock staging distance; <=0 means use the scalar staging_distance.
        sdists = list(self.get_parameter('dock_staging_dist').value)
        self.dock_stg    = {i: (d if d > 0.0 else self.stg_dist)
                            for i, d in zip(ids, sdists)}
        self.seq_dwell   = self.get_parameter('sequence_dwell').value
        self.undock_back_scale = self.get_parameter('undock_back_scale').value

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        # ── State ─────────────────────────────────────────────────────────────
        self.phase      = 'idle'   # idle|staging|approach|pivot|final|docked|undock
        self.target_id  = None
        self.target_tag = None     # selected AprilTag frame
        self._nav_handle = None
        self._last_mile = False    # True once we've handed staging off to SmacHybrid
        self._undock_start = None  # (x, y) in fixed_frame at undock start
        self._last_dist  = None    # last fresh tag distance (m), for blind creep
        self._lost_since = None    # clock time the tag was first lost mid-approach
        self._creep_start = None   # (x, y) in fixed_frame at blind-creep start
        self._creep_remaining = 0.0
        self._pivot_dock = None    # (x, y) in map the search pivot rotates toward
        self._pivot_dir  = 1       # +1 forward leg, -1 reverse leg
        self._pivot_seg_start = None  # (x, y) at the current K-turn leg's start
        self._pivot_start_t = None
        self._undock_target_dist = self.undock_dist  # set per-undock
        # Docking sequence ("go to dock A, then B, ..."): runs each dock in turn,
        # undocking to ~staging between IF currently docked (else drives straight on).
        self._seq        = []      # remaining dock ids to visit
        self._seq_idx    = 0
        self._seq_loop   = False   # cycle back to the first when the list ends
        self._seq_active = False
        self._seq_wait   = None    # 'dock' | 'undock' | 'settle' | None
        self._seq_dwell_start = None
        self._settle_start = None  # clock time the inter-dock settle/clear started

        # ── I/O ───────────────────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Costmap "reset scan" services — used to re-seed the costmaps from a fresh
        # laser scan after undocking, so the dock just left isn't a stale obstacle.
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        # Latched state topic so a poller always sees the current phase.
        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/docking_state', state_qos)

        # Latched planner selection -> the BT's PlannerSelector. Latched (transient
        # local) so a late-joining / restarted bt_navigator still gets the current
        # choice. Default to the cheap transit planner until we start staging.
        self.planner_pub = self.create_publisher(String, planner_sel_topic, state_qos)
        self._select_planner(self.transit_planner)

        self.create_subscription(String, '/dock_robot',     self._trigger_cb,   10)
        self.create_subscription(String, '/dock_sequence',  self._sequence_cb,  10)
        self.create_subscription(Bool,   '/undock_robot',   self._undock_cb,    10)
        self.create_subscription(Bool,   '/abort_docking',  self._abort_cb,     10)

        self.create_timer(0.05, self._loop)           # 20 Hz control loop
        self.create_timer(0.2,  self._publish_state)   # 5 Hz state heartbeat
        self._publish_state()

        self.get_logger().info(
            'jetracer_docker ready. docks=%s. /dock_robot to dock, '
            '/undock_robot to back out, /abort_docking to stop.'
            % list(self.dock_to_tag))

    # ── State helpers ─────────────────────────────────────────────────────────

    def _set_phase(self, p):
        self.phase = p
        self._publish_state()

    def _publish_state(self):
        self.state_pub.publish(String(data=self.phase))

    def _select_planner(self, planner_id):
        """Tell the BT which loaded Nav2 planner to use on the next replan."""
        self.planner_pub.publish(String(data=planner_id))

    def _clear_costmaps(self):
        """Re-seed both Nav2 costmaps from the current scan ("reset scan").

        Fire-and-forget: if a costmap server isn't up we just skip it rather than
        block the control loop.
        """
        for name, client in (('local', self.clear_local),
                             ('global', self.clear_global)):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(
                    f'{name}_costmap clear service not ready — skipping reset.')

    def _begin_settle(self):
        """Stop, clear the costmaps, and start the inter-dock settle timer."""
        self._stop()
        self._clear_costmaps()
        self._settle_start = self.get_clock().now()
        self.get_logger().info(
            f'[seq] settling {self.settle_time:.0f}s + clearing costmaps before '
            'next dock.')

    def _settle_done(self):
        if self._settle_start is None:
            return True
        elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
        return elapsed >= self.settle_time

    def _fail_dock(self):
        """Abort the current dock attempt: stop and return to idle.

        Leaves any active sequence running — the sequence supervisor sees phase
        'idle' with no dock and advances to the next target on its own.
        """
        self._stop()
        self._last_dist = None
        self._lost_since = None
        self._set_phase('idle')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _trigger_cb(self, msg):
        if self.phase not in ('idle', 'docked'):
            self.get_logger().warn(f'Busy ({self.phase}) — ignoring dock request.')
            return
        self._seq_active = False   # a manual single dock cancels any sequence
        self._begin_dock(msg.data.strip())

    def _begin_dock(self, dock_id):
        """Start docking the given dock id (staging -> approach)."""
        tag = self.dock_to_tag.get(dock_id)
        if tag is None:
            self.get_logger().error(
                f'Unknown dock id "{dock_id}". Known: {list(self.dock_to_tag)}.')
            self._seq_active = False
            return
        self.target_id  = dock_id
        self.target_tag = tag
        self._last_dist = None
        self._lost_since = None
        self.get_logger().info(f'Dock requested: {dock_id} (tag frame {tag}).')
        if self.skip_stg:
            self._set_phase('approach')
        else:
            self._start_staging()

    def _undock_cb(self, msg):
        if not msg.data:
            return
        if self.phase in ('staging', 'approach', 'undock'):
            self.get_logger().warn(f'Busy ({self.phase}) — ignoring undock request.')
            return
        self._start_undock()

    def _abort_cb(self, _msg):
        self.get_logger().warn('Abort received.')
        self._seq_active = False
        self._seq_wait = None
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None
        self._select_planner(self.transit_planner)
        self._stop()
        self._set_phase('idle')

    # ── Docking sequence ──────────────────────────────────────────────────────

    def _sequence_cb(self, msg):
        """Start a sequence like "dock0,dock2" (append ",loop" to cycle)."""
        if self.phase in ('staging', 'approach', 'pivot', 'final', 'undock'):
            self.get_logger().warn(f'Busy ({self.phase}) — ignoring sequence.')
            return
        parts = [p.strip() for p in msg.data.split(',') if p.strip()]
        loop = bool(parts) and parts[-1].lower() in ('loop', 'cycle')
        if loop:
            parts = parts[:-1]
        parts = [p for p in parts if p in self.dock_to_tag]
        if not parts:
            self.get_logger().error('Empty/invalid dock sequence.')
            return
        self._seq        = parts
        self._seq_idx    = 0
        self._seq_loop   = loop
        self._seq_active = True
        self._seq_dwell_start = None
        self.get_logger().info(f'[seq] starting {self._seq} loop={loop}.')
        if self.phase == 'docked':
            # Already against a dock -> back out toward staging before navigating.
            self._start_undock()
            self._seq_wait = 'undock'
        else:
            self._begin_dock(self._seq[0])
            self._seq_wait = 'dock'

    def _seq_next_index(self):
        n = self._seq_idx + 1
        if n < len(self._seq):
            return n
        return 0 if self._seq_loop else None

    def _seq_advance(self, was_docked):
        """Move to the next dock; undock first only if we actually docked."""
        nxt = self._seq_next_index()
        if nxt is None:
            self.get_logger().info('[seq] sequence complete.')
            self._seq_active = False
            self._seq_wait = None
            return
        self._seq_idx = nxt
        if was_docked:
            # Physically against the dock -> back out toward staging, then dock next.
            self._start_undock()
            self._seq_wait = 'undock'
        else:
            # Not docked (open space) -> drive straight to the next dock.
            self._begin_dock(self._seq[self._seq_idx])
            self._seq_wait = 'dock'

    def _sequence_supervise(self):
        """Drive the sequence forward off phase transitions (runs each tick)."""
        if not self._seq_active:
            return
        if self._seq_wait == 'dock':
            if self.phase == 'docked':
                # Dwell while docked, then move on (undocking first).
                if self._seq_dwell_start is None:
                    self._seq_dwell_start = self.get_clock().now()
                    self.get_logger().info(
                        f'[seq] docked at {self.target_id}; dwell {self.seq_dwell:.0f}s.')
                elif (self.get_clock().now() - self._seq_dwell_start).nanoseconds / 1e9 \
                        >= self.seq_dwell:
                    self._seq_dwell_start = None
                    self._seq_advance(was_docked=True)
            elif self.phase == 'idle':
                # Dock attempt ended without docking (pivot timeout / lost) -> the car
                # is in open space, so go to the next dock directly, no undock.
                self.get_logger().warn(
                    f'[seq] {self.target_id} did not dock; moving on without undock.')
                self._seq_advance(was_docked=False)
            # staging/approach/pivot/final -> still working; keep waiting.
        elif self._seq_wait == 'undock':
            if self.phase == 'idle':
                # Backed out to ~staging. Don't drive at the next dock yet: settle
                # first (clear costmaps + pause) so we never replan forward into
                # the dock we just left while it's still right in front of us.
                self._seq_wait = 'settle'
                self._begin_settle()
        elif self._seq_wait == 'settle':
            if self._settle_done():
                self._settle_start = None
                self._seq_wait = 'dock'
                self._begin_dock(self._seq[self._seq_idx])

    # ── Phase 1: Staging via Nav2 ─────────────────────────────────────────────

    def _dock_position_in_map(self):
        """(x, y) of the target dock in map: surveyed pose if set, else tag TF."""
        xy = self.dock_xy.get(self.target_id)
        if xy is not None and (xy[0] != 0.0 or xy[1] != 0.0):
            return xy
        try:
            dm = self.tf.lookup_transform('map', self.target_tag, rclpy.time.Time())
            return (dm.transform.translation.x, dm.transform.translation.y)
        except Exception:
            return None

    def _start_staging(self):
        dock = self._dock_position_in_map()
        if dock is None:
            self.get_logger().warn('No map pose and tag not visible — skipping staging.')
            self._set_phase('approach')
            return

        # Approach axis: the robot's heading when docked (facing into the dock).
        # Prefer the surveyed dock heading so the staging pose sits squarely in
        # front of the dock on its approach axis; fall back to facing the dock
        # from wherever the robot currently is (robot->dock line).
        theta = self.dock_yaw.get(self.target_id)
        if theta is None:
            try:
                robot_map = self.tf.lookup_transform(
                    'map', self.base_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f'TF robot→map failed: {e}. Skipping staging.')
                self._set_phase('approach')
                return
            dx = dock[0] - robot_map.transform.translation.x
            dy = dock[1] - robot_map.transform.translation.y
            if math.hypot(dx, dy) < 1e-3:
                self.get_logger().warn('Dock too close to stage. Skipping staging.')
                self._set_phase('approach')
                return
            theta = math.atan2(dy, dx)

        # Place the staging pose this dock's staging distance back from the dock
        # along theta, facing in. Nav2 just needs to get the car ~here (loose goal
        # tolerance); the pivot/approach handle the precision.
        stg = self.dock_stg.get(self.target_id, self.stg_dist)
        staging = PoseStamped()
        staging.header.frame_id = 'map'
        staging.header.stamp    = self.get_clock().now().to_msg()
        staging.pose.position.x = dock[0] - math.cos(theta) * stg
        staging.pose.position.y = dock[1] - math.sin(theta) * stg
        staging.pose.position.z = 0.0
        staging.pose.orientation = self._yaw_to_quat(theta)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 not available. Skipping staging.')
            self._set_phase('approach')
            return

        goal = NavigateToPose.Goal()
        goal.pose = staging
        self.get_logger().info(
            f'Staging to ({staging.pose.position.x:.2f}, {staging.pose.position.y:.2f})')
        # Drive the long approach on the cheap forward-only transit planner (smooth,
        # not jerky). _staging_feedback hands off to the Hybrid-A* staging planner
        # for the last_mile_dist stretch, where the heading-aware reversing lineup
        # into the dock is needed. If last_mile_dist <= 0, switch immediately.
        self._last_mile = False
        if self.last_mile_dist > 0.0:
            self._select_planner(self.transit_planner)
        else:
            self._select_planner(self.staging_planner)
            self._last_mile = True
        self._set_phase('staging')
        self.nav_client.send_goal_async(
            goal, feedback_callback=self._staging_feedback
        ).add_done_callback(self._staging_sent_cb)

    def _staging_feedback(self, feedback_msg):
        """Hand off transit -> staging planner once within last_mile_dist of the goal.

        distance_remaining is the length of the current global path to the staging
        pose. The BT's PlannerSelector reads the latched planner id on its next 1 Hz
        replan, so republishing here switches the planner seamlessly mid-goal. Latched
        via _last_mile so we never flip back to the transit planner on this leg.
        """
        if self._last_mile or self.phase != 'staging':
            return
        # distance_remaining is 0.0 before the first path is computed -- ignore that
        # sentinel so we don't hand off before the transit leg even starts.
        dist = feedback_msg.feedback.distance_remaining
        if 0.0 < dist <= self.last_mile_dist:
            self._last_mile = True
            self.get_logger().info(
                f'Last mile ({self.last_mile_dist:.2f} m) — switching to '
                f'{self.staging_planner} for the heading-aware dock approach.')
            self._select_planner(self.staging_planner)

    def _staging_sent_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Staging goal rejected.')
            self._select_planner(self.transit_planner)
            self._set_phase('idle')
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._staging_done_cb)

    def _staging_done_cb(self, future):
        self._nav_handle = None
        # Staging navigation is over -> restore the cheap transit planner as default.
        self._select_planner(self.transit_planner)
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Staging done. Starting approach.')
        else:
            self.get_logger().warn(f'Staging failed (status={status}). Approaching anyway.')
        self._set_phase('approach')

    # ── Phase 2: Pure Pursuit approach ────────────────────────────────────────

    def _loop(self):
        self._sequence_supervise()
        if self.phase == 'approach':
            self._approach()
        elif self.phase == 'pivot':
            self._pivot_step()
        elif self.phase == 'final':
            self._final_step()
        elif self.phase == 'undock':
            self._undock_step()

    def _approach(self):
        # Selected tag pose in base_footprint (latest available).
        tf = None
        try:
            tf = self.tf.lookup_transform(
                self.base_frame, self.target_tag, rclpy.time.Time())
            age = (self.get_clock().now()
                   - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age > self.det_tmo:
                tf = None   # stale
        except (LookupException, ConnectivityException, ExtrapolationException):
            tf = None

        if tf is None:
            self._handle_lost()
            return

        x = tf.transform.translation.x   # forward
        y = tf.transform.translation.y   # left
        dist = math.hypot(x, y)
        self._last_dist = dist           # remember for blind creep
        self._lost_since = None          # fresh detection -> reset the lost timer

        if dist < self.dock_thr:
            self.get_logger().info(f'Docked at {self.target_id} ({dist:.3f} m).')
            self._stop()
            self._set_phase('docked')
            return

        if x < 0.05:
            self.get_logger().warn('Dock behind robot — stopping.')
            self._stop()
            return

        alpha = math.atan2(y, x)
        steer = self.steer_gain * math.atan2(2.0 * self.L * math.sin(alpha), dist)
        steer = max(-self.delta_max, min(self.delta_max, steer))
        omega = self.speed * math.tan(steer) / self.L

        cmd = Twist()
        cmd.linear.x  = self.speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def _handle_lost(self):
        """Tag not visible/fresh during approach."""
        # Lost while already close -> the tag has just left the FOV; finish the
        # remaining distance open-loop on odometry, then declare docked.
        if self._last_dist is not None and self._last_dist <= self.blind_dist:
            self._creep_remaining = max(0.0, self._last_dist - self.dock_thr)
            try:
                t = self.tf.lookup_transform(
                    self.fixed_frame, self.base_frame, rclpy.time.Time())
                self._creep_start = (t.transform.translation.x,
                                     t.transform.translation.y)
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().warn('Tag lost close in but no odom — declaring docked.')
                self._stop()
                self._set_phase('docked')
                return
            self.get_logger().info(
                f'Tag left FOV at {self._last_dist:.2f} m — creeping '
                f'{self._creep_remaining:.2f} m blind to finish docking.')
            self._set_phase('final')
            return
        # Never acquired the tag (just arrived at the staging pose and it is not in
        # the camera FOV): pivot in place toward the dock to bring it into view.
        if self._last_dist is None:
            self._start_pivot()
            return
        # Lost mid-approach while still far: hold and wait for re-detection, but
        # not forever — give up after approach_lost_timeout so a failed dock can't
        # wedge the node (and a sequence can move on to the next dock).
        self._stop()
        now = self.get_clock().now()
        if self._lost_since is None:
            self._lost_since = now
        waited = (now - self._lost_since).nanoseconds / 1e9
        if self.approach_lost_tmo > 0.0 and waited >= self.approach_lost_tmo:
            self.get_logger().error(
                f'Dock {self.target_id}: tag not re-detected in '
                f'{self.approach_lost_tmo:.0f}s — aborting attempt.')
            self._fail_dock()
            return
        self.get_logger().warn(
            'Waiting for dock detection…', throttle_duration_sec=2.0)

    def _final_step(self):
        """Open-loop straight creep for the last stretch after the tag is lost."""
        try:
            t = self.tf.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        dx = t.transform.translation.x - self._creep_start[0]
        dy = t.transform.translation.y - self._creep_start[1]
        if math.hypot(dx, dy) >= self._creep_remaining:
            self.get_logger().info(f'Docked at {self.target_id} (blind creep).')
            self._stop()
            self._set_phase('docked')
            return
        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

    # ── Search pivot: multi-point turn to bring the tag into view ──────────────

    def _start_pivot(self):
        dock = self._dock_position_in_map()
        if dock is None:
            self._stop()
            self.get_logger().warn(
                'No tag and no dock map pose to pivot toward — waiting.',
                throttle_duration_sec=2.0)
            return
        self._pivot_dock = dock
        self._pivot_dir  = 1
        self._pivot_seg_start = None
        self._pivot_start_t = self.get_clock().now()
        self.get_logger().info(
            'No tag at staging — multi-point pivoting toward the dock to find it.')
        self._set_phase('pivot')

    def _pivot_step(self):
        # Tag found mid-pivot -> hand straight off to the pure-pursuit approach.
        if self._tag_visible():
            self.get_logger().info('Tag acquired during pivot — approaching.')
            self._stop()
            self._set_phase('approach')
            return

        # Never spin forever.
        if (self.get_clock().now() - self._pivot_start_t).nanoseconds / 1e9 \
                > self.pivot_timeout:
            self.get_logger().warn('Pivot timed out without the tag — aborting.')
            self._stop()
            self._set_phase('idle')
            return

        try:
            t = self.tf.lookup_transform('map', self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        rx, ry = t.transform.translation.x, t.transform.translation.y
        err = self._wrap(
            math.atan2(self._pivot_dock[1] - ry, self._pivot_dock[0] - rx)
            - self._yaw_from_quat(t.transform.rotation))

        # Facing the dock but tag still unseen -> not detectable from here; hold.
        if abs(err) < self.pivot_align_tol:
            self._stop()
            self.get_logger().warn(
                'Pivot faces the dock but tag not seen — waiting.',
                throttle_duration_sec=2.0)
            return

        s = 1.0 if err > 0.0 else -1.0     # rotate toward reducing err

        # Advance the current leg; flip forward/reverse each segment so the heading
        # keeps rotating one way while net translation stays ~zero (a K-turn).
        if self._pivot_seg_start is None:
            self._pivot_seg_start = (rx, ry)
        if math.hypot(rx - self._pivot_seg_start[0],
                      ry - self._pivot_seg_start[1]) >= self.pivot_seg_dist:
            self._pivot_dir *= -1
            self._pivot_seg_start = (rx, ry)

        # Forward leg steers +s, reverse leg steers -s: both turn the heading by s,
        # giving the S / 3-point shape (the steer flips with the drive direction).
        v     = self.pivot_speed * self._pivot_dir
        steer = s * self._pivot_dir * self.delta_max
        # angular.z must encode the STEERING direction, not the signed yaw rate: the
        # firmware steers by sign(angular.z) (see _approach, which feeds positive
        # speed). Using signed v here double-flips and steers the SAME way on both
        # legs -> the car just retraces one arc. Build omega from the positive speed
        # so the steer sign carries through and the wheels flip on the reverse leg.
        omega = self.pivot_speed * math.tan(steer) / self.L

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    # ── Undock: reverse straight back ─────────────────────────────────────────

    def _start_undock(self):
        try:
            t = self.tf.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF for undock failed: {e}. Aborting undock.')
            return
        self._undock_start = (t.transform.translation.x, t.transform.translation.y)
        # Reverse back to ~the staging pose: the dock's staging distance minus the
        # docking threshold (we stop that close to the tag). Fall back to the fixed
        # undock_distance if this dock has no staging distance / we aren't at a dock.
        stg = self.dock_stg.get(self.target_id) if self.target_id else None
        base = max(0.1, stg - self.dock_thr) if stg else self.undock_dist
        self._undock_target_dist = base * self.undock_back_scale
        self.get_logger().info(
            f'Undocking — reversing {self._undock_target_dist:.2f} m toward staging.')
        self._set_phase('undock')

    def _undock_step(self):
        try:
            t = self.tf.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        dx = t.transform.translation.x - self._undock_start[0]
        dy = t.transform.translation.y - self._undock_start[1]
        if math.hypot(dx, dy) >= self._undock_target_dist:
            self.get_logger().info('Undock complete.')
            self._stop()
            self._set_phase('idle')
            return
        cmd = Twist()
        cmd.linear.x = -self.undock_speed
        self.cmd_pub.publish(cmd)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _yaw_to_quat(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _tag_visible(self):
        """True if the selected tag has a fresh TF in the base frame."""
        try:
            tf = self.tf.lookup_transform(
                self.base_frame, self.target_tag, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        age = (self.get_clock().now()
               - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        return age <= self.det_tmo

    @staticmethod
    def _yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = JetRacerDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
