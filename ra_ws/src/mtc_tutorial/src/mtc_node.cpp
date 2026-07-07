#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <cmath>
#include <limits>
#include <atomic>
#include <mutex>
#include <map>
#include <chrono>
#include <functional>
#include <Eigen/Geometry>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

// ── Reusable incremental (velocity-form) PID ─────────────────────────────────
// Returns the per-step INCREMENT to ADD to a position command (a joint angle).
// Both centering loops do `command += step(error)` each tick, and accumulating an
// incremental PID yields a true POSITIONAL PID on the command:
//   Σ kp*(e-e_prev)            = kp*e         → proportional
//   Σ ki*e                     = ki*Σe        → integral (kills steady-state error)
//   Σ kd*(e-2e_prev+e_prev2)   = kd*(e-e_prev)→ derivative (damps overshoot)
// du = kp*(e - e1) + ki*e + kd*(e - 2*e1 + e2),  clamped to ±max_step.
// Discrete (per-sample, dt-free): fine for these slow, roughly fixed-rate loops.
// The OLD pure-proportional behaviour is exactly ki=<old gain>, kp=kd=0.
struct IncrementalPid
{
  double kp{ 0.0 }, ki{ 0.0 }, kd{ 0.0 };
  double max_step{ 1e9 };
  double e1{ 0.0 }, e2{ 0.0 };   // e(k-1), e(k-2)
  int    n{ 0 };                 // samples seen (gates kp/kd until history exists)

  void reset() { e1 = e2 = 0.0; n = 0; }

  double step(double e)
  {
    double du = ki * e;
    if (n >= 1) du += kp * (e - e1);
    if (n >= 2) du += kd * (e - 2.0 * e1 + e2);
    e2 = e1; e1 = e; if (n < 2) ++n;
    return std::max(-max_step, std::min(max_step, du));
  }
};

// Level side-grasp wrist pitch, CLAMPED to just inside the URDF limit (±1.65806).
// The raw rule -(Pitch+Elbow) can command the wrist a hair PAST its bound; the
// low-level controller tracks it anyway (it doesn't enforce URDF planning limits),
// leaving the arm in a state MTC's CheckStartStateBounds rejects ("Start state out
// of bounds"). The URDF limit is 1.65806. A ~0.03° margin was NOT enough: Isaac's
// tracking overshoot + joint-state noise still crossed the bound intermittently,
// failing the next MTC plan (e.g. the place-release "open hand"). Keep a real
// margin (~0.038 rad / 2.2°) so the wrist can never park on the hard limit; the
// reach cost at full saturation is a few mm — imperceptible for the grasp/place.
static constexpr double WRIST_PITCH_LIMIT = 1.62;
static inline double levelWristPitch(double pitch, double elbow)
{
  return std::max(-WRIST_PITCH_LIMIT, std::min(WRIST_PITCH_LIMIT, -(pitch + elbow)));
}

// Arm positioning joints, in the controller/URDF order used everywhere below.
static const std::vector<std::string> ARM_JOINTS =
    { "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll" };

// Assemble a level side-grasp joint goal: positioning joints (Rotation, Pitch,
// Elbow) come from IK; the wrist is tied to keep the gripper horizontal
// (Wrist_Pitch = -(Pitch+Elbow), clamped) with the given roll about the approach.
static inline std::map<std::string, double> levelGoal(double R, double P, double E,
                                                      double wrist_roll)
{
  return {
    { "Rotation", R }, { "Pitch", P }, { "Elbow", E },
    { "Wrist_Pitch", levelWristPitch(P, E) }, { "Wrist_Roll", wrist_roll }
  };
}

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();
  // Place the "object" cup collision cylinder at a world position (doTask pins it
  // once at the settled detection). A helper so the cylinder dims live in one spot.
  void applyObjectCollision(double x, double y, double z);

  // Post-pick flow: carry the grasped cup home, then approach the AprilTag-marked
  // dispenser paddle (standoff along the tag normal, then advance in to "fill"),
  // and place it in the tray. cup_index / num_cups: which cup of the batch this is,
  // so it's offset into a distinct tray slot (multi-cup refill).
  void continueToDispenser(int cup_index = 0, int num_cups = 1);

private:
  mtc::Task createTask();
  // ── continueToDispenser sub-steps (see their definitions for details) ─────────
  // Wait up to timeout_s for the first /apriltag/pose latch; false if none arrives.
  bool latchTagPose(geometry_msgs::msg::PoseStamped& tag, double timeout_s = 5.0);
  // MTC pick: close gripper, attach object, lift straight up (level wrist rule).
  bool runPickLiftTask(double lift_height);
  // MTC release: open gripper + detach object (arm holds still).
  bool runReleaseTask();
  // Clamp the arm inside its URDF joint bounds + re-command, so the next MTC task's
  // CurrentState validates (Isaac can settle a hair past a limit). No-op if in bounds.
  void reseatArmInBounds();
  // Resolve the tray place point for cup_index/num_cups (detected-or-param centre +
  // systematic offset + auto/explicit multi-cup slot); fills place_yaw and place_z.
  void resolveTrayPlacePoint(int cup_index, int num_cups, double& place_x,
                             double& place_y, double& place_z, double& place_yaw);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  // ── Object pose from perception ──────────────────────────────────────────
  std::mutex obj_mutex_;
  double obj_x_{ 0.0 };
  double obj_y_{ 0.0 };
  double obj_z_{ 0.0 };
  std::atomic<bool> obj_received_{ false };
  // Wall-clock of the last detection (freshness gate: don't servo on stale data
  // if the hand camera loses the blob mid-approach).
  rclcpp::Time obj_stamp_{ 0, 0, RCL_ROS_TIME };
  bool obj_have_stamp_{ false };
  // EMA-filtered object estimate used by the servo, to damp the per-tick jitter
  // the eye-in-hand view induces. Raw obj_x_/obj_y_ stay unfiltered for logging.
  double obj_fx_{ 0.0 }, obj_fy_{ 0.0 };
  bool obj_filt_init_{ false };

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obj_sub_;

  // ── Destination tray pose from perception (pink tray, top_cam) ────────────
  // Static target (fixed tray), so we just latch the latest fix — no filtering.
  // The place stage reads this to drop the cup at the DETECTED tray centre
  // instead of the hard-coded place_x/place_y.
  std::mutex tray_mutex_;
  double tray_x_{ 0.0 }, tray_y_{ 0.0 }, tray_z_{ 0.0 };
  std::atomic<bool> tray_received_{ false };
  rclcpp::Time tray_stamp_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tray_sub_;

  // ── All platform cups (world) for collision avoidance ─────────────────────
  // Every detected platform cup; the pre-grasp planner adds the NON-target ones
  // as collision cylinders so it routes around the cup(s) it isn't picking.
  std::mutex cups_mutex_;
  std::vector<Eigen::Vector3d> cups_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cups_sub_;
  std::vector<std::string> cup_obstacle_ids_;   // ids currently in the scene
  void applyCupObstacles(double target_x, double target_y);

  // ── Object detection in IMAGE space (for image-based servoing) ────────────
  // /detected_object/pixel = [u, v, img_w, img_h, bbox_w, bbox_h]. Closing the
  // servo loop on these pixel errors (rather than the unprojected world point)
  // is robust to a wrong hand-eye extrinsic — no extrinsic/depth in the loop.
  std::mutex px_mutex_;
  double px_u_{ 0.0 }, px_v_{ 0.0 }, px_iw_{ 0.0 }, px_ih_{ 0.0 };
  double px_bh_{ 0.0 };  // bbox height (used for apparent-size); bbox width unused
  rclcpp::Time px_stamp_{ 0, 0, RCL_ROS_TIME };
  bool px_have_{ false };
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr px_sub_;

  // /detected_object/depth = perceived metric range (m) from the ACTIVE camera to
  // the detection centre (the camera-frame depth). During the eye-in-hand servo
  // (active_camera = arm_cam) this is the live arm-cam range to the cup, used by
  // phase 1 to halt the straight-in approach the instant the cup is within reach.
  std::mutex depth_mutex_;
  double arm_depth_{ 0.0 };
  rclcpp::Time depth_stamp_{ 0, 0, RCL_ROS_TIME };
  bool arm_depth_have_{ false };
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;

  // ── AprilTag (dispenser paddle marker) world pose, from apriltag_node ──────
  // Published on /apriltag/pose by so_arm_perception/apriltag_node, ideally off
  // the fixed top_cam (eye-to-hand) so the world fix is stable regardless of arm
  // pose — letting continueToDispenser() latch it at task-build time.
  std::mutex tag_mutex_;
  geometry_msgs::msg::PoseStamped tag_pose_world_;
  bool tag_have_{ false };
  rclcpp::Time tag_stamp_{ 0, 0, RCL_ROS_TIME };  // arrival time (freshness gate)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_sub_;

  // ── Visual servo (interlaced with MTC) ───────────────────────────────────
  // Closed-loop level side grasp: position-only /compute_ik (seeded from the
  // current state) places the gripper at the object, then the wrist rule
  // (Wrist_Roll=-pi/2, Wrist_Pitch=-(Pitch+Elbow)) keeps it a horizontal side
  // grasp with the object held upright.
  void visualServoToGrasp();
  // Eye-in-hand: switch the perception node's source camera at runtime so the
  // servo consumes object positions from the hand-mounted camera ("arm_cam")
  // during alignment, and the overhead camera ("top_cam") otherwise.
  void setActiveCamera(const std::string& cam);
  rclcpp::AsyncParametersClient::SharedPtr perception_param_client_;
  // Switch the apriltag node's source camera at runtime (arm_cam for closed-loop
  // tag centering after the cup is grasped). Mirrors setActiveCamera.
  void setAprilTagCamera(const std::string& cam);
  rclcpp::AsyncParametersClient::SharedPtr apriltag_param_client_;
  void servoTick();
  // Image-based servo: drive the integrators from pixel error and emit the claw-
  // midpoint world target. Returns false (HOLD) on stale/missing pixels.
  bool computeImageTarget(double& ox, double& oy, double& target_z, double& yaw);
  void ikDone(rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future,
              sensor_msgs::msg::JointState::SharedPtr seed);
  // Phase-1 Cartesian approach: sample a straight world line to the grasp
  // distance, solve position-only IK + the level wrist rule per sample, and
  // stream it as ONE time-parameterized trajectory. Returns false if no waypoint
  // was reachable (caller falls back to the per-tick integrator).
  bool issuePhase1Cartesian(const sensor_msgs::msg::JointState::SharedPtr& js);
  // Stream a straight-line, LEVEL gripper move from the current pose to a world
  // gripper-origin target, using the same per-waypoint position-only IK + level
  // side-grasp wrist rule as phase 1 — so a carried cup stays upright (the SRDF
  // "home" rolls the wrist to 0, which would tip it). Publishes one trajectory to
  // cmd_pub_ and returns its duration (s) so the caller can wait it out.
  double streamLevelMove(const Eigen::Vector3d& target_g);
  // Parameter readers: return the param if already set, else declare it with the
  // default (matches automatically_declare_parameters_from_overrides). One per type.
  double      paramd(const std::string& n, double d);
  bool        paramb(const std::string& n, bool d);
  std::string params(const std::string& n, const std::string& d);
  // Build a RobotState seeded from a joint-state message / the latest /joint_states.
  moveit::core::RobotState seedState(const moveit::core::RobotModelConstPtr& rm,
                                     const sensor_msgs::msg::JointState& js);
  moveit::core::RobotState seedStateFromCurrent(const moveit::core::RobotModelConstPtr& rm);
  // Open the claw directly via the gripper controller (blocking on the action
  // result), instead of an MTC MoveTo — avoids planning/start-state validation for
  // a gripper-only move. position defaults to the SRDF open_grip Jaw value.
  bool openGripper(double position = 1.7453);
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  // Kept ALIVE for the node's lifetime: the loader owns the kinematics-plugin
  // class loader, so robot_model_'s solver objects outlive it. A throwaway loader
  // would be destroyed while those objects exist → the class_loader "unload while
  // objects exist" warning.
  std::shared_ptr<robot_model_loader::RobotModelLoader> rml_;
  moveit::core::RobotModelConstPtr robot_model_;  // lazily loaded for in-process IK
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  // Latched RViz cup marker — published when the cup position is received so RViz
  // visibly shows it (add a Marker display on /detected_object/cup_marker).
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cup_marker_pub_;
  // Claw-midpoint (TCP) marker: a sphere fixed in the `gripper` frame at the SAME
  // offset the grasp math uses (tcp_lat, tcp_z, -tcp_fwd in gripper-local coords),
  // so RViz shows exactly where the code aims the "middle of the gripper". Add a
  // Marker display on /claw_tcp_marker. Republished on a timer so it tracks live.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tcp_marker_pub_;
  rclcpp::TimerBase::SharedPtr tcp_marker_timer_;
  void publishTcpMarker();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr align_sub_;
  std::mutex js_mutex_;
  sensor_msgs::msg::JointState::SharedPtr current_js_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  // Reentrant group so the async /compute_ik response callback runs while the
  // subscriptions keep being serviced (MultiThreadedExecutor).
  rclcpp::CallbackGroup::SharedPtr servo_cb_group_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  std::atomic<bool> ik_busy_{ false };
  std::atomic<bool> servo_done_{ false };
  int servo_settled_{ 0 };
  // Cached servo params (set in visualServoToGrasp, read in tick/response cb).
  double s_standoff_{ 0.0 }, s_gain_{ 0.6 }, s_max_step_{ 0.30 };
  double s_wrist_roll_{ -M_PI / 2.0 }, s_grasp_z_{ 0.2 };
  // Desired base Rotation (rad) streamLevelMove should PIN the arm to, so position-
  // only IK can't drift the base off the side-grasp facing branch (which left the
  // carried cup pointing away from the dispenser). For a level side-grasp the base
  // Rotation = gripper facing yaw + pi/2. NaN = let IK choose (carry-over warm start).
  double s_level_seed_rotation_{ std::numeric_limits<double>::quiet_NaN() };
  double s_settle_tol_{ 0.01 };  // commanded step below this = arm has settled
  double s_obj_alpha_{ 0.5 };    // EMA weight on new sample (1=no filter)
  double s_max_obj_age_{ 0.4 };  // s; older detection = stale, hold instead of chase
  // Tool-center-point offset: the claw midpoint (where the object should end up)
  // relative to the gripper/Wrist_Roll origin that IK actually places. In the
  // level side-grasp pose the jaws straddle the object centered, so only a
  // forward (along approach) and vertical component are non-zero.
  double s_tcp_fwd_{ 0.05 }, s_tcp_z_{ 0.0 };
  // Lateral TCP offset (m) ⊥ to the facing yaw. The SO-101 has ONE moving jaw;
  // the object is pinned against the FIXED jaw, off to one side of the gap. This
  // shifts the grasp spot to the fixed-jaw face (+ = robot-left). 0 = gap centre.
  double s_tcp_lat_{ 0.0 };
  // Approach yaw bias (rad): rotates the facing so the object lands IN THE GAP
  // between the fixed and moving jaw (not dead-centre on the closed gripper).
  // Runtime-tunable mirror of YAW_RIGHT_BIAS (default -0.4 ≈ 23° right; try
  // -0.785 for 45°). Shared by gross-move, servo, place, dispenser so all agree.
  double s_yaw_bias_{ -0.4 };
  // PHASE-1-ONLY extra pull-back (m): the straight-in approach STOPS this far short
  // of the grasp distance, so the gripper travels less toward the cup. Raise it to
  // shorten the phase-1 reach (e.g. if it overshoots / pushes the cup). On top of
  // s_standoff_. Launch-tunable via `servo_img_p1_pullback`.
  double s_img_p1_pullback_{ 0.02 };
  // PHASE-1-ONLY lateral bias on the approach TARGET (m). Shifts the end point
  // perpendicular to the cup bearing: + = LEFT of the approach heading, - = right.
  // Use to nudge the grasp slightly off the detected centre. Launch-tunable via
  // `servo_img_p1_lateral`.
  double s_img_p1_lateral_{ 0.0 };
  // PHASE-1-ONLY scale on the approach yaw bias (s_yaw_bias_). 1.0 = full jaw-gap
  // bias (gripper heads in ~29° off the cup bearing); <1 reduces how far the
  // straight-in direction is biased off straight-at-the-cup (0 = dead straight).
  // Only the phase-1 approach uses this; the bridge/phase-0 facing keep the full
  // s_yaw_bias_. Launch-tunable via `servo_img_p1_yaw_scale`.
  double s_img_p1_yaw_scale_{ 0.5 };
  // PHASE-1 depth-stop: when true, the straight-in approach is HALTED the instant
  // the live arm_cam perceived range (/detected_object/depth) drops below
  // s_img_p1_depth_stop_m. This closes the loop on the camera (robust to a wrong
  // open-loop reach estimate: it stops at the truly-perceived standoff). The
  // smooth one-shot cartesian glide still runs underneath; depth just stops it
  // early (and the joint-arrival test stays as the fallback if depth never
  // crosses / drops out). Launch-tunable via `servo_img_p1_depth_stop[_m]`.
  bool   s_img_p1_depth_stop_{ false };
  double s_img_p1_depth_stop_m_{ 0.12 };
  // Max age (s) of a depth reading that may trigger the stop (reject stale ranges).
  double s_img_p1_depth_timeout_{ 0.5 };
  bool   img_p1_halted_{ false };  // latched once the depth-stop has issued the halt

  // ── Servo smoothing ───────────────────────────────────────────────────────
  // The async /compute_ik cadence is slow (seconds/reply). Publishing each point
  // with a fixed short horizon made the arm dart to the target then freeze until
  // the next reply (stop-go = "spasming"). Stretch the horizon to span the real
  // command period so the controller glides between replies; low-pass the joints
  // to damp position-only-IK branch flips.
  double s_cmd_time_{ 0.2 };       // s; horizon floor / default before timing
  double s_cmd_horizon_k_{ 1.2 };  // horizon = k * measured inter-command period
  double s_cmd_smooth_{ 1.0 };     // EMA alpha on published joints (1 = off)
  rclcpp::Time last_cmd_stamp_;
  bool have_last_cmd_{ false };
  std::map<std::string, double> cmd_filt_;
  bool cmd_filt_init_{ false };

  // ── Image-based (IBVS) servo gains ────────────────────────────────────────
  // The servo target is driven by arm-cam IMAGE pixel error: horizontal error →
  // bearing (base Rotation), apparent bbox size / world distance → approach reach.
  // Integrators are seeded from the coarse top_cam detection, then refined purely
  // in image space — robust to a wrong arm-cam extrinsic.
  double s_img_k_yaw_{ -0.0015 };   // rad of bearing per pixel of horizontal error (sign matches arm-cam mount)
  // Phase-0 bearing PID (cup centering). ki defaults to s_img_k_yaw_ so kp=kd=0
  // reproduces the old pure-proportional law; add kp/kd to damp the approach.
  IncrementalPid img_yaw_pid_;
  double s_img_k_reach_{ 0.25 };    // easing gain on reach toward the metric grasp distance
  double s_img_deadband_px_{ 6.0 }; // ignore pixel errors smaller than this
  double s_img_done_px_{ 12.0 };    // centered within this = aligned
  // Stop-on-center: when true, the servo HALTS the instant the blob is centered
  // (phase 0) instead of handing off to the phase-1 straight-in approach — for
  // when phase 1 drives too far forward into the object.
  bool   s_img_stop_on_center_{ false };
  // Horizontal setpoint offset from image center, in pixels (+ = aim RIGHT of
  // center). For a side grasp the gripper isn't centered on the camera, so the
  // object should converge slightly off-center, not dead center.
  double s_img_u_offset_px_{ 0.0 };
  double s_img_reach_min_{ 0.08 }, s_img_reach_max_{ 0.35 };
  double s_img_standback_{ 0.07 };  // m held BEHIND the grasp distance during phase 0,
                                    // so phase 0 centers at a standoff and phase 1 does
                                    // the real straight-in approach (creeps this gap)
  // Per-tick motion caps so the servo makes SMALL adjustments (big sweeps at
  // grasp distance knock the object over / cause left-right hunting).
  double s_img_yaw_step_{ 0.006 };  // max rad of bearing change per tick (small:
                                    // one step must move the object less than the
                                    // centered band or the loop limit-cycles)
  // Phase-0 turn-speed taper: within s_img_slow_px_ of the setpoint, scale the
  // per-tick yaw step down linearly (to a floor) so the base SLOWS as the object
  // approaches centre instead of charging at full step until the deadband.
  double s_img_slow_px_{ 150.0 };
  double s_img_slow_floor_{ 0.2 };  // min fraction of yaw_step kept inside the zone
  double s_img_slow_pow_{ 2.0 };    // taper curve exponent: 1=linear, >1 decelerates
                                    // HARDER as the object nears centre (slower creep)
  double s_img_reach_step_{ 0.02 }; // max m of reach change per tick
  double s_img_bearing_range_{ 0.8 };// max rad the bearing may swing from its seed
  // Anti-windup (back-calculation) strength. Scales the phase-0 integrator step
  // down by s_img_aw_ * (joint tracking error / align_tol), so the bearing
  // command can't wind far ahead of the lagging arm and overshoot centre. Only
  // modulates the rate (never freezes — floored at 0.1); 0 = plain integrator.
  double s_img_aw_{ 1.0 };
  bool   img_have_world_cap_{ false };// a real world fix has set img_reach_cap_ at least once
  // True while the eye-in-hand (arm_cam) servo is active. During eih we DO NOT
  // let the live world fix refresh img_reach_cap_: the arm_cam depth+TF world
  // estimate is the unreliable quantity the IBVS design distrusts, and a too-
  // close reading collapses the grasp distance below the current reach so phase 1
  // has nothing to approach ("doesn't move forward"). The cap is instead frozen
  // at the accurate top_cam seed taken before the camera switch.
  bool   eih_active_{ false };
  // Integrator state (live target in polar-about-base form).
  double img_bearing_{ 0.0 }, img_reach_{ 0.2 }, img_tz_{ 0.2 };
  double img_bearing0_{ 0.0 }, img_tz0_{ 0.2 };  // seeds, for bound clamping
  // Hard metric cap on how far forward the claw midpoint may travel, derived
  // from the world-seed object distance minus the standoff. The bbox-size
  // approach is otherwise unbounded by real distance and will ram the gripper
  // straight through the object; this guarantees it stops at/short of the object.
  double img_reach_cap_{ 0.35 };
  int img_settled_{ 0 };
  // Two-phase image servo: PHASE 0 = VISUAL left/right alignment (bearing from
  // pixels, reach frozen — never advances toward the object, so it can't knock it
  // over while centering); PHASE 1 = POSITION straight-in approach (bearing
  // frozen, reach eased toward the metric grasp distance). Advance 0→1 once the
  // object is centered AND the arm has caught up (joint max_err < align_tol);
  // declare done in PHASE 1 once max_err < approach_tol.
  int img_phase_{ 0 };
  double s_img_align_tol_{ 0.5 };     // rad; joint max_err to finish visual alignment
  double s_img_approach_tol_{ 0.2 };  // rad; joint max_err to finish straight approach
  int    s_img_align_ticks_{ 2 };     // centered net-ticks to confirm alignment (leaky:
                                       // a brief off-center tick decays, not resets, the
                                       // count — robust to the residual centering jitter)
  std::atomic<double> servo_max_err_{ 1e9 };  // last joint-space max_err (ikDone → align gate)

  // ── Phase-1 Cartesian approach (toggle) ─────────────────────────────────────
  // Instead of the per-tick reach integrator + async IK, phase 1 can stream ONE
  // straight-line Cartesian approach: a smooth, continuous, time-parameterized
  // trajectory along the frozen bearing to the grasp distance. Avoids the per-tick
  // position-only IK branch flips and the single-point stop-go.
  bool   s_img_p1_cartesian_{ true };  // phase 1 = streamed Cartesian (vs integrator)
  double s_img_cart_speed_{ 0.06 };    // m/s EE speed along the approach line (carry/lean)
  double s_img_cart_step_{ 0.01 };     // m between sampled Cartesian waypoints
  bool   img_p1_issued_{ false };      // the approach trajectory has been published
  std::map<std::string, double> img_p1_goal_;  // final commanded joints (arrival test)
};

// Convert a desired CLAW-MIDPOINT world point into the world target for the
// gripper (Wrist_Roll) IK link, given the level side-grasp orientation (gripper
// level, yawed toward the object). The claw midpoint sits `tcp_fwd` ahead of the
// gripper origin along the approach direction and `tcp_z` above it, so to land
// the claws on the spot the gripper origin must be placed that much back/down.
// Convert a desired grasp spot (world) to the gripper-origin IK target.
//  - tcp_fwd : how far the contact point sits FORWARD of the gripper origin
//              along the facing yaw (claw reaches ahead of the link).
//  - tcp_lat : lateral shift PERPENDICULAR to yaw. The SO-101 gripper has ONE
//              moving jaw; the object is pinned against the FIXED jaw, which is
//              offset to one side of the gap centre. tcp_lat references the spot
//              to that fixed-jaw face (not the gap midpoint), so the spot ends up
//              against the fixed jaw with the opening on the moving-jaw side.
//              +tcp_lat shifts the gripper toward robot-LEFT (REP-103 +90° of
//              yaw); flip the sign for the other side. 0 = old gap-centre aim.
static inline void clawMidpointToGripperTarget(
    double spot_x, double spot_y, double spot_z, double yaw,
    double tcp_fwd, double tcp_z, double& gx, double& gy, double& gz,
    double tcp_lat = 0.0)
{
  // Lateral unit vector = yaw rotated +90° (robot-left): (-sin, cos).
  gx = spot_x - tcp_fwd * std::cos(yaw) - tcp_lat * std::sin(yaw);
  gy = spot_y - tcp_fwd * std::sin(yaw) + tcp_lat * std::cos(yaw);
  gz = spot_z - tcp_z;
}
static constexpr double CUP_HEIGHT = 0.09;
static constexpr double CUP_RADIUS = 0.03;
static constexpr double TABLE_Z    = 0.0;
// Real mug geometry for the grasp HEIGHT (CUP_HEIGHT above is the padded
// collision cylinder, not the true mug). Aim the side grasp at the geometric
// middle of the mug body, independent of the detector's vertical centroid bias.
static constexpr double MUG_HEIGHT  = 0.09;                          // true mug height
static constexpr double CUP_BASE_Z  = 0.01486;                       // mug base z on the table (sim)
static constexpr double CUP_GRASP_Z = CUP_BASE_Z + MUG_HEIGHT / 2.0; // 0.05986, mid-height
static constexpr double ELEVATED    = 0.15;
// Aim slightly to the RIGHT of the object center (not dead at it). REP-103:
// positive yaw is CCW = robot's LEFT, so a NEGATIVE bias rotates the aim to the
// RIGHT. Applied EVERYWHERE we face the object (gross move AND servo) so the two
// phases agree on where "facing the object" points. -0.4 rad ≈ 23° right.
static constexpr double YAW_RIGHT_BIAS = -0.4;
// ── Constructor ─────────────────────────────────────────────────────────────
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // Subscribe to perception output
  obj_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_object/position", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(obj_mutex_);
        obj_x_ = msg->point.x;
        obj_y_ = msg->point.y;
        // Freshness: stamp the arrival so the servo can reject stale detections.
        obj_stamp_ = node_->now();
        obj_have_stamp_ = true;
        // EMA-filter the estimate the servo consumes (damps eye-in-hand jitter).
        if (!obj_filt_init_)
        {
          obj_fx_ = obj_x_;
          obj_fy_ = obj_y_;
          obj_filt_init_ = true;
        }
        else
        {
          obj_fx_ = s_obj_alpha_ * obj_x_ + (1.0 - s_obj_alpha_) * obj_fx_;
          obj_fy_ = s_obj_alpha_ * obj_y_ + (1.0 - s_obj_alpha_) * obj_fy_;
        }
        // Object height straight from the (now-calibrated) top_cam detection:
        // /detected_object/position.z is the unprojected world z of the detection
        // (~1 mm accurate), so use it directly instead of a hardcoded elevated-
        // surface guess. Feeds the grasp height (s_grasp_z_, snapshotted at servo
        // start) and the collision cylinder centre. Override with `servo_grasp_z`.
        obj_z_ = msg->point.z;
        if (!obj_received_.load())
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Object detected: x=%.4f  y=%.4f  z=%.4f",
                      obj_x_, obj_y_, obj_z_);
        }
        obj_received_.store(true);
      });

  // Destination tray (pink) world position from perception. Latest fix latched;
  // the place stage reads it to place the cup at the detected tray centre instead
  // of the hard-coded place_x/place_y. Static target → no EMA/filtering needed.
  tray_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_tray/position", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(tray_mutex_);
        tray_x_ = msg->point.x;
        tray_y_ = msg->point.y;
        tray_z_ = msg->point.z;
        tray_stamp_ = node_->now();
        if (!tray_received_.load())
          RCLCPP_INFO(node_->get_logger(),
                      "Tray detected: x=%.4f  y=%.4f  z=%.4f",
                      tray_x_, tray_y_, tray_z_);
        tray_received_.store(true);
      });

  // All platform cups (world frame) for collision avoidance. Latest list latched;
  // applyCupObstacles() adds the non-target cups to the planning scene each cycle.
  cups_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/detected_cups", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cups_mutex_);
        cups_.clear();
        for (const auto& p : msg->poses)
          cups_.emplace_back(p.position.x, p.position.y, p.position.z);
      });

  // Image-space detection (for image-based servoing). Stamp arrival so the servo
  // can reject stale pixels the same way it does world fixes.
  px_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/detected_object/pixel", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        std::lock_guard<std::mutex> lock(px_mutex_);
        px_u_  = msg->data[0]; px_v_  = msg->data[1];
        px_iw_ = msg->data[2]; px_ih_ = msg->data[3];
        px_bh_ = msg->data[5];  // data[4] = bbox width, unused
        px_stamp_ = node_->now();
        px_have_ = true;
      });

  // Perceived metric range (m) to the detection from the active camera. Phase 1
  // uses the eye-in-hand (arm_cam) value to stop the straight-in approach once the
  // cup is within servo_img_p1_depth_stop_m. Stamp arrival for a freshness gate.
  depth_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/detected_object/depth", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        arm_depth_ = msg->data;
        depth_stamp_ = node_->now();
        arm_depth_have_ = true;
      });

  // AprilTag world pose (dispenser paddle marker). Latest fix latched by
  // continueToDispenser() to drive the standoff + fill approach.
  tag_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/apriltag/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(tag_mutex_);
        tag_pose_world_ = *msg;
        tag_stamp_ = node_->now();
        tag_have_ = true;
      });

  // ── Visual-servo I/O ───────────────────────────────────────────────────────
  servo_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_group_controller/joint_trajectory", 10);
  // Gripper (Jaw) GripperCommand action client — opens the claw directly via the
  // hand_group_controller (see moveit_controllers.yaml), bypassing an MTC plan.
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      node_, "/hand_group_controller/gripper_cmd", servo_cb_group_);
  // Latched (transient_local) so an RViz that connects later still gets the cup.
  cup_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/detected_object/cup_marker", rclcpp::QoS(1).transient_local());
  // Claw-midpoint (TCP) visualization. DEFAULT ON; disable with show_tcp_marker:=false.
  // The sphere lives in the `gripper` frame so RViz moves it with the arm; a ~15 Hz
  // timer republishes it (with a fresh stamp) so it tracks smoothly during motion.
  const bool show_tcp = paramb("show_tcp_marker", true);
  if (show_tcp)
  {
    tcp_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/claw_tcp_marker", rclcpp::QoS(1).transient_local());
    tcp_marker_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(66), [this]() { publishTcpMarker(); });
  }
  rclcpp::SubscriptionOptions js_opts;
  js_opts.callback_group = servo_cb_group_;
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(js_mutex_);
        current_js_ = msg;
      },
      js_opts);
  ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>(
      "/compute_ik", rmw_qos_profile_services_default, servo_cb_group_);

  // Client to flip the perception node's `active_camera` (eye-in-hand servoing).
  perception_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, node_->declare_parameter("perception_node_name",
                                      std::string("/perception_node")));

  // Client to flip the apriltag node's `active_camera` (top_cam for the coarse
  // latch, arm_cam for closed-loop tag centering after the cup is grasped).
  apriltag_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, node_->declare_parameter("apriltag_node_name",
                                      std::string("/apriltag_node")));

  // Optional: topic published by camera/perception node indicating the
  // detected object is aligned/visible from the hand-mounted camera. When
  // a `std_msgs/Bool` with `data==true` is received, the servo loop will
  // stop immediately.
  std::string align_topic = node_->declare_parameter("servo_align_topic",
                                                     std::string("/detected_object/aligned"));
  if (!align_topic.empty())
  {
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = servo_cb_group_;
    align_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        align_topic, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data)
          {
            RCLCPP_INFO(node_->get_logger(), "[servo] alignment detected via %s, stopping servo.",
                        node_->get_parameter("servo_align_topic").as_string().c_str());
            servo_done_.store(true);
          }
        },
        opt);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Waiting for object position on /detected_object/position ...");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// ── Parameter readers ────────────────────────────────────────────────────────
double MTCTaskNode::paramd(const std::string& n, double d)
{
  return node_->has_parameter(n) ? node_->get_parameter(n).as_double()
                                 : node_->declare_parameter<double>(n, d);
}
bool MTCTaskNode::paramb(const std::string& n, bool d)
{
  return node_->has_parameter(n) ? node_->get_parameter(n).as_bool()
                                 : node_->declare_parameter<bool>(n, d);
}
std::string MTCTaskNode::params(const std::string& n, const std::string& d)
{
  return node_->has_parameter(n) ? node_->get_parameter(n).as_string()
                                 : node_->declare_parameter<std::string>(n, d);
}

// ── RobotState seeding ───────────────────────────────────────────────────────
// A RobotState at default values, overwritten with the positions from `js`.
moveit::core::RobotState MTCTaskNode::seedState(
    const moveit::core::RobotModelConstPtr& rm, const sensor_msgs::msg::JointState& js)
{
  moveit::core::RobotState rs(rm);
  rs.setToDefaultValues();
  if (!js.name.empty())
  {
    std::map<std::string, double> seed;
    for (size_t i = 0; i < js.name.size(); ++i) seed[js.name[i]] = js.position[i];
    rs.setVariablePositions(seed);
  }
  rs.update();
  return rs;
}
// Same, seeded from the latest /joint_states under the lock.
moveit::core::RobotState MTCTaskNode::seedStateFromCurrent(
    const moveit::core::RobotModelConstPtr& rm)
{
  sensor_msgs::msg::JointState js;
  { std::lock_guard<std::mutex> lk(js_mutex_); if (current_js_) js = *current_js_; }
  return seedState(rm, js);
}

// ── Open the claw via the gripper controller ─────────────────────────────────
// Sends a GripperCommand goal straight to hand_group_controller (no MTC plan) and
// blocks on the result. The executor spins on another thread, so waiting on the
// action futures here doesn't deadlock. Returns false only if the server is
// unavailable or the goal is rejected (a non-SUCCEEDED result — e.g. the controller
// reporting "stalled" once open — is logged but not treated as fatal).
bool MTCTaskNode::openGripper(double position)
{
  using GripperCommand = control_msgs::action::GripperCommand;
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(LOGGER, "[gripper] action server /hand_group_controller/gripper_cmd "
                 "unavailable — cannot open claw");
    return false;
  }
  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 0.0;   // 0 = controller default
  RCLCPP_INFO(LOGGER, "[gripper] opening claw (Jaw -> %.4f)", position);

  auto gh_future = gripper_client_->async_send_goal(goal);
  if (gh_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  { RCLCPP_ERROR(LOGGER, "[gripper] open: goal send timed out"); return false; }
  auto goal_handle = gh_future.get();
  if (!goal_handle) { RCLCPP_ERROR(LOGGER, "[gripper] open: goal REJECTED"); return false; }

  auto res_future = gripper_client_->async_get_result(goal_handle);
  if (res_future.wait_for(std::chrono::seconds(8)) != std::future_status::ready)
  { RCLCPP_WARN(LOGGER, "[gripper] open: result timed out — continuing"); return true; }
  const auto wrapped = res_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    RCLCPP_WARN(LOGGER, "[gripper] open: action did not SUCCEED (code %d) — continuing",
                static_cast<int>(wrapped.code));
  return true;
}

// Place the "object" cup collision cylinder at (x,y,z).
void MTCTaskNode::applyObjectCollision(double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { CUP_HEIGHT, CUP_RADIUS };  // height, radius

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;  // cylinder center = half height above table
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  // Visual update in RViz: a green cylinder marker at the same pose/dims, so the
  // cup is plainly visible (the planning-scene collision object can be easy to
  // miss). Latched, so RViz shows it whenever it connects.
  if (cup_marker_pub_)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = node_->now();
    m.ns = "cup";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose;
    m.scale.x = m.scale.y = 2.0 * CUP_RADIUS;  // diameter
    m.scale.z = CUP_HEIGHT;                     // height
    m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2; m.color.a = 0.8;
    cup_marker_pub_->publish(m);
  }
}

// Add the NON-target platform cups (from /detected_cups) to the planning scene as
// collision cylinders, so the pre-grasp planner routes around the cup(s) it isn't
// currently picking. Clears the previous cycle's obstacles first. The cup nearest
// (target_x, target_y) — i.e. the one being grasped, published separately as
// "object" — is skipped so it doesn't double as an obstacle blocking its own grasp.
// NOTE: only the MTC pre-grasp is collision-checked; the visual-servo final
// approach and the streamed carry/place are NOT, so they still ignore these.
void MTCTaskNode::applyCupObstacles(double target_x, double target_y)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Remove last cycle's obstacle cylinders.
  if (!cup_obstacle_ids_.empty())
  {
    psi.removeCollisionObjects(cup_obstacle_ids_);
    cup_obstacle_ids_.clear();
  }

  std::vector<Eigen::Vector3d> cups;
  { std::lock_guard<std::mutex> lock(cups_mutex_); cups = cups_; }

  // Radius within which a listed cup is considered "the target" (skip it).
  const double match_r = paramd("cup_obstacle_match_radius", 0.05);

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  int k = 0;
  for (const auto& c : cups)
  {
    if (std::hypot(c.x() - target_x, c.y() - target_y) <= match_r)
      continue;  // this is the target cup ("object"), not an obstacle
    moveit_msgs::msg::CollisionObject o;
    o.id = "cup_obstacle_" + std::to_string(k++);
    o.header.frame_id = "world";
    o.primitives.resize(1);
    o.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    o.primitives[0].dimensions = { CUP_HEIGHT, CUP_RADIUS };  // height, radius
    o.pose.position.x = c.x();
    o.pose.position.y = c.y();
    o.pose.position.z = c.z();
    o.pose.orientation.w = 1.0;
    o.operation = moveit_msgs::msg::CollisionObject::ADD;
    objs.push_back(o);
    cup_obstacle_ids_.push_back(o.id);
  }
  if (!objs.empty())
  {
    psi.applyCollisionObjects(objs);
    RCLCPP_INFO(LOGGER, "[collision] added %zu cup obstacle(s) around target (%.3f,%.3f)",
                objs.size(), target_x, target_y);
  }
}

// Publish the claw-midpoint (TCP) as a sphere in the `gripper` frame. The offset
// is the SAME one the grasp math uses (clawMidpointToGripperTarget): in gripper-
// local coords the world (tcp_fwd forward, tcp_z up, tcp_lat lateral) maps to
// (tcp_lat, tcp_z, -tcp_fwd) — because at the level side grasp the gripper's local
// -z is the horizontal "forward" (facing) axis and local +y is world-up (verified
// by FK). RViz transforms the marker via TF, so it rides the live arm and lands on
// exactly the point the approach trajectory is planned for.
void MTCTaskNode::publishTcpMarker()
{
  if (!tcp_marker_pub_) return;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "gripper";
  m.header.stamp = node_->now();
  m.ns = "claw_tcp";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = s_tcp_lat_;
  m.pose.position.y = s_tcp_z_;
  m.pose.position.z = -s_tcp_fwd_;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = 0.012;   // 12 mm ball
  m.color.r = 1.0; m.color.g = 0.2; m.color.b = 1.0; m.color.a = 0.9;  // magenta
  tcp_marker_pub_->publish(m);
}

// ── Planning scene ───────────────────────────────────────────────────────────
void MTCTaskNode::setupPlanningScene()
{
  // Object collision cylinder is NOT placed here — doTask() pins it ONCE at the
  // settled, filtered detection (a single update, no live tracking). Placing it
  // here from the 0.0 defaults previously put a phantom cup at the world origin
  // ("cup below the arm at start"); leaving it out means no cup appears until the
  // grasp position is committed.
  moveit::planning_interface::PlanningSceneInterface psi;
  // ── Table surface ─────────────────────────────────────────────────────────
  // A thin box whose TOP face sits at world z = 0. We make the box 2 cm thick,
  // so its center is at z = -0.01. The arm body is collision-checked against
  // this, which is what keeps the SO-ARM from swinging into the table.
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "world";
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 0.44, 0.44, 0.02 };  // x, y, thickness

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.13722;
  table_pose.position.y = -0.15386;
  table_pose.position.z = TABLE_Z - 0.01;  // top face flush with z = 0
  table_pose.orientation.w = 1.0;
  table.pose = table_pose;

  psi.applyCollisionObject(table);
}

// ── Execute task ─────────────────────────────────────────────────────────────
void MTCTaskNode::doTask()
{
  // ── Mode ───────────────────────────────────────────────────────────────────
  // mode:  hybrid (MTC gross move → visual-servo bridge → dispenser) | servo (test).
  // The cup COUNT is NOT a param — it's how many the top cam detects (below);
  // perception's tray-region exclusion then keeps each placed cup from being
  // re-detected, so each cycle locks onto the NEXT platform cup.
  std::string mode = "hybrid";
  if (node_->has_parameter("mode"))
    mode = node_->get_parameter("mode").as_string();

  // ── Servo test mode is single-cup: wait for one fix, servo, return ─────────
  if (mode == "servo")
  {
    RCLCPP_INFO(LOGGER, "Blocking until /detected_object/position is received ...");
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !obj_received_.load()) rate.sleep();
    if (!rclcpp::ok()) return;
    double sx, sy, sz;
    { std::lock_guard<std::mutex> lock(obj_mutex_); sx = obj_x_; sy = obj_y_; sz = obj_z_; }
    applyObjectCollision(sx, sy, sz);
    RCLCPP_INFO(LOGGER, "==== SERVO TEST MODE — closed-loop level side grasp ====");
    visualServoToGrasp();
    return;
  }

  // ── Scale the run to how many cups the TOP CAM sees ────────────────────────
  // The total comes from /detected_cups (top_cam), so the tray layout CENTRES a
  // lone cup and spreads N cups across N slots automatically. If the top cam
  // reports none (topic silent / nothing on the platform), default to ONE cup.
  // Switch perception to the top cam and clear the stale list so the count is a
  // FRESH top-cam view (a restarted node could inherit arm_cam from a prior run).
  setActiveCamera(params("servo_idle_camera", "top_cam"));
  { std::lock_guard<std::mutex> lk(cups_mutex_); cups_.clear(); }
  int total_cups = 1;
  {
    rclcpp::Rate rate(10);
    for (int i = 0; i < 30 && rclcpp::ok(); ++i)   // up to ~3 s for the first cup list
    {
      { std::lock_guard<std::mutex> lk(cups_mutex_); if (!cups_.empty()) break; }
      rate.sleep();
    }
    size_t detected = 0;
    { std::lock_guard<std::mutex> lk(cups_mutex_); detected = cups_.size(); }
    if (detected > 0)
    {
      total_cups = static_cast<int>(detected);
      RCLCPP_INFO(LOGGER, "Top cam detected %zu cup(s) — placing across %d tray slot(s)",
                  detected, total_cups);
    }
    else
      RCLCPP_WARN(LOGGER, "Top cam reported no cups — defaulting to 1 cup");
  }

  // ── Multi-cup loop: detect → (MTC pre-grasp) → servo → fill/place → home ───
  for (int cup = 0; cup < total_cups && rclcpp::ok(); ++cup)
  {
    RCLCPP_INFO(LOGGER, "==== ========== CUP %d / %d ========== ====", cup + 1, total_cups);

    // Wait for a FRESH detection. Reset the latch + EMA filter so we never reuse
    // the previous cup's pose; perception's tray-region exclusion means the cup
    // already sitting in the tray is skipped, so this locks onto the next platform cup.
    {
      std::lock_guard<std::mutex> lk(obj_mutex_);
      obj_received_.store(false);
      obj_filt_init_ = false;
    }
    RCLCPP_INFO(LOGGER, "Blocking until /detected_object/position (cup %d/%d) ...",
                cup + 1, total_cups);
    {
      rclcpp::Rate rate(10);
      while (rclcpp::ok() && !obj_received_.load()) rate.sleep();
    }
    if (!rclcpp::ok()) return;

    double snap_x, snap_y, snap_z;
    { std::lock_guard<std::mutex> lock(obj_mutex_); snap_x = obj_x_; snap_y = obj_y_; snap_z = obj_z_; }
    RCLCPP_INFO(LOGGER,
                "Cup %d/%d snapped — x=%.4f y=%.4f z=%.4f — creating task ...",
                cup + 1, total_cups, snap_x, snap_y, snap_z);
    applyObjectCollision(snap_x, snap_y, snap_z);
    // Add the OTHER platform cups as collision obstacles so the pre-grasp plan
    // routes around them (target cup is skipped inside).
    applyCupObstacles(snap_x, snap_y);

    // Open the claw via the gripper controller BEFORE the pre-grasp move (replaces
    // the old MTC "open hand" stage), so the arm arrives at the standoff open.
    openGripper();

    task_ = createTask();
    try { task_.enableIntrospection(); task_.init(); }
    catch (mtc::InitStageException& e) { RCLCPP_ERROR_STREAM(LOGGER, e); return; }

    RCLCPP_INFO(LOGGER, "==== [1] Planning (cup %d/%d) ====", cup + 1, total_cups);
    if (!task_.plan(1))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      auto& stages = *task_.stages();
      for (size_t i = 0; i < stages.numChildren(); ++i)
      {
        const auto* stage = stages.operator[](i);
        RCLCPP_INFO(LOGGER, "Stage [%s]: %zu solutions, %zu failures",
                    stage->name().c_str(), stage->solutions().size(), stage->failures().size());
        for (const auto& s : stage->solutions())
          RCLCPP_INFO(LOGGER, "  solution: cost=%.4f comment='%s'", s->cost(), s->comment().c_str());
        for (const auto& f : stage->failures())
        {
          RCLCPP_INFO(LOGGER, "  failure: comment='%s'", f->comment().c_str());
          task_.introspection().publishSolution(*f);
        }
      }
      return;
    }
    RCLCPP_INFO(LOGGER, "==== [1] Planning SUCCEEDED — %zu solution(s) ====",
                task_.solutions().size());

    const auto& solution = *task_.solutions().front();
    RCLCPP_INFO(LOGGER, "==== [3] Publishing solution to RViz (cost=%.4f) ====", solution.cost());
    task_.introspection().publishSolution(solution);

    RCLCPP_INFO(LOGGER, "==== [4] Executing ====");
    auto result = task_.execute(solution);
    RCLCPP_INFO(LOGGER, "Execution error code: %d", result.val);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code " << result.val);
      return;
    }
    RCLCPP_INFO(LOGGER, "==== [4] Execution SUCCEEDED ====");

    // The collision-checked pre-grasp move is done, so drop the cup obstacles:
    // the servo/pick phases that follow are NOT collision-checked, and a nearby
    // obstacle could otherwise flag their start state as in-collision.
    if (!cup_obstacle_ids_.empty())
    {
      moveit::planning_interface::PlanningSceneInterface psi;
      psi.removeCollisionObjects(cup_obstacle_ids_);
      cup_obstacle_ids_.clear();
    }

    // Gross move reached — close the loop with the visual servo, then carry the cup
    // to the dispenser and place it in the tray.
    RCLCPP_INFO(LOGGER, "==== [5] Pre-grasp reached — starting visual servo bridge ====");
    visualServoToGrasp();
    RCLCPP_INFO(LOGGER, "==== [6] Visual servo bridge complete ====");
    continueToDispenser(cup, total_cups);
  }

  RCLCPP_INFO(LOGGER, "==== ALL %d CUP(S) COMPLETE ====", total_cups);
}

// ── continueToDispenser sub-steps ────────────────────────────────────────────
// Wait (up to timeout_s) for the first /apriltag/pose latch. top_cam is fixed
// (eye-to-hand) so its tag->world fix is valid regardless of arm pose; we capture
// it UP FRONT, before any motion, while the dispenser is clearly in view (once the
// arm moves to home it can occlude the top_cam's line to the tag).
bool MTCTaskNode::latchTagPose(geometry_msgs::msg::PoseStamped& tag, double timeout_s)
{
  rclcpp::Rate rate(10);
  const int ticks = std::max(1, static_cast<int>(timeout_s * 10.0));
  for (int i = 0; i < ticks && rclcpp::ok(); ++i)
  {
    { std::lock_guard<std::mutex> lk(tag_mutex_);
      if (tag_have_) { tag = tag_pose_world_; return true; } }
    rate.sleep();
  }
  RCLCPP_ERROR(LOGGER,
      "[dispenser] no /apriltag/pose received — is apriltag_node running, the "
      "tag in top_cam's view, and TF top_sim_camera->world up? Aborting.");
  return false;
}

// MTC pick: close the gripper, attach the object, and lift straight up by
// lift_height with the level side-grasp wrist rule (keeps the cup upright).
bool MTCTaskNode::runPickLiftTask(double lift_height)
{
  mtc::Task task;
  task.stages()->setName("pick + home");
  task.loadRobotModel(node_);
  task.setProperty("group",    "arm_group");
  task.setProperty("eef",      "gripper");
  task.setProperty("ik_frame", "gripper");

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("ompl", "RRTConnectkConfigDefault");
  const double arm_speed = paramd("arm_speed", 0.75);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(arm_speed);
  interpolation_planner->setMaxAccelerationScalingFactor(arm_speed);

  { auto stage = std::make_unique<mtc::stages::CurrentState>("current"); task.add(std::move(stage)); }
  { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object",
        task.getRobotModel()->getJointModelGroup("hand_group")->getLinkModelNamesWithCollisionGeometry(), true);
    task.add(std::move(stage)); }
  { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
    stage->allowCollisions("object", "table", true); task.add(std::move(stage)); }
  { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,table)");
    stage->allowCollisions("table",
        task.getRobotModel()->getJointModelGroup("hand_group")->getLinkModelNamesWithCollisionGeometry(), true);
    task.add(std::move(stage)); }

  auto robot_model = task.getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup("arm_group");

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    { auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup("hand_group"); stage->setGoal("close_grip"); grasp->insert(std::move(stage)); }
    { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", "gripper"); grasp->insert(std::move(stage)); }
    {
      moveit::core::RobotState rs = seedStateFromCurrent(robot_model);
      Eigen::Isometry3d target = rs.getGlobalLinkTransform("gripper");
      target.translation().z() += lift_height;
      if (!rs.setFromIK(jmg, target, "gripper", 0.1))
        RCLCPP_ERROR(LOGGER, "[dispenser] lift position-only IK failed for z+%.3f m", lift_height);
      const double R = rs.getVariablePosition("Rotation");
      const double P = rs.getVariablePosition("Pitch");
      const double E = rs.getVariablePosition("Elbow");
      auto stage = std::make_unique<mtc::stages::MoveTo>("lift object", sampling_planner);
      stage->setGroup("arm_group"); stage->setGoal(levelGoal(R, P, E, -M_PI / 2.0));
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  try { task.enableIntrospection(); task.init(); }
  catch (mtc::InitStageException& e) { RCLCPP_ERROR_STREAM(LOGGER, e); return false; }
  RCLCPP_INFO(LOGGER, "==== [7a] Planning pick + lift ====");
  if (!task.plan(1)) { RCLCPP_ERROR(LOGGER, "[dispenser] pick + lift planning failed"); return false; }
  RCLCPP_INFO(LOGGER, "==== [7a] Executing pick + lift ====");
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  { RCLCPP_ERROR(LOGGER, "[dispenser] pick + lift execution failed (%d)", result.val); return false; }
  RCLCPP_INFO(LOGGER, "==== [7a] Pick + lift SUCCEEDED ====");
  return true;
}

// Re-seat the arm strictly INSIDE its URDF joint bounds before an MTC task that
// builds off CurrentState: the streamed place-lower can leave Isaac's controller
// settled a fraction of a degree past a limit, which MoveIt rejects as "Start state
// out of bounds". (1) wait for rest, (2) clamp each joint release_bounds_margin
// inside its bound and re-command, (3) wait for it to settle there.
void MTCTaskNode::reseatArmInBounds()
{
  if (!robot_model_) return;
  const double margin = paramd("release_bounds_margin", 0.02);

  auto snapshot = [&](std::map<std::string, double>& out) -> bool {
    std::map<std::string, double> live;
    { std::lock_guard<std::mutex> lk(js_mutex_);
      if (current_js_ && !current_js_->name.empty())
        for (size_t i = 0; i < current_js_->name.size(); ++i)
          live[current_js_->name[i]] = current_js_->position[i]; }
    out.clear();
    for (const auto& j : ARM_JOINTS) {
      auto it = live.find(j);
      if (it == live.end()) return false;
      out[j] = it->second;
    }
    return true;
  };

  // (1) Wait for the arm to come to rest (two consecutive reads within tol).
  {
    rclcpp::Rate r(20);
    std::map<std::string, double> prev, cur;
    bool have_prev = snapshot(prev);
    const auto t0 = node_->now();
    while (rclcpp::ok() && (node_->now() - t0).seconds() < 1.5)
    {
      r.sleep();
      if (!snapshot(cur)) { have_prev = false; continue; }
      if (have_prev) {
        double md = 0.0;
        for (const auto& j : ARM_JOINTS) md = std::max(md, std::abs(cur[j] - prev[j]));
        if (md < s_settle_tol_) break;
      }
      prev = cur; have_prev = true;
    }
  }

  // (2) Snapshot the rest pose and clamp each joint inside its URDF bound.
  std::map<std::string, double> seated;
  if (!snapshot(seated)) return;
  bool clamped = false;
  for (const auto& j : ARM_JOINTS)
  {
    const auto& b = robot_model_->getVariableBounds(j);
    if (!b.position_bounded_) continue;
    const double lo = b.min_position_ + margin;
    const double hi = b.max_position_ - margin;
    const double cv = std::min(hi, std::max(lo, seated[j]));
    if (std::abs(cv - seated[j]) > 1e-6) {
      RCLCPP_WARN(LOGGER, "[dispenser] release re-seat: %s %.4f -> %.4f (bound %.4f..%.4f)",
                  j.c_str(), seated[j], cv, b.min_position_, b.max_position_);
      seated[j] = cv; clamped = true;
    }
  }
  if (!clamped) return;  // already in bounds

  // (3) Re-command the clamped pose and wait for it to settle there.
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = ARM_JOINTS;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  for (const auto& j : ARM_JOINTS) pt.positions.push_back(seated[j]);
  pt.time_from_start = rclcpp::Duration::from_seconds(0.4);
  traj.points.push_back(pt);
  cmd_pub_->publish(traj);
  RCLCPP_INFO(LOGGER, "[dispenser] re-seating arm inside bounds before release");
  rclcpp::Rate r(20);
  const auto t0 = node_->now();
  while (rclcpp::ok() && (node_->now() - t0).seconds() < 2.0)
  {
    std::map<std::string, double> cur;
    if (snapshot(cur)) {
      double md = 0.0;
      for (const auto& j : ARM_JOINTS) md = std::max(md, std::abs(cur[j] - seated[j]));
      if (md < s_settle_tol_) break;
    }
    r.sleep();
  }
}

// Release: open the claw via the gripper controller, then detach the cup in the
// planning scene (a tiny MTC task; the arm holds still). Only the OPEN uses the
// gripper controller directly — the detach stays MTC for the scene bookkeeping.
bool MTCTaskNode::runReleaseTask()
{
  RCLCPP_INFO(LOGGER, "==== [10] Open claw (gripper controller) + detach ====");
  if (!openGripper()) return false;

  mtc::Task task;
  task.stages()->setName("place release (detach)");
  task.loadRobotModel(node_);
  task.setProperty("group",    "arm_group");
  task.setProperty("eef",      "gripper");
  task.setProperty("ik_frame", "gripper");
  { auto stage = std::make_unique<mtc::stages::CurrentState>("current"); task.add(std::move(stage)); }
  { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
    stage->allowCollisions("object", "table", true); task.add(std::move(stage)); }
  { auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("object", "gripper"); task.add(std::move(stage)); }
  try { task.enableIntrospection(); task.init(); }
  catch (mtc::InitStageException& e) { RCLCPP_ERROR_STREAM(LOGGER, e); return false; }
  if (!task.plan(1)) { RCLCPP_ERROR(LOGGER, "[dispenser] detach planning failed"); return false; }
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  { RCLCPP_ERROR(LOGGER, "[dispenser] detach execution failed (%d)", result.val); return false; }
  return true;
}

// Resolve the pink-tray place point. Start from the detected tray centre
// (/detected_tray/position) or the place_x/place_y params, add the measured
// systematic offset, then for multi-cup spread each cup into a distinct slot
// (explicit place_slot_ys, else auto-spread along the tray long axis). Fills the
// facing yaw (place_yaw) and release height (place_z = tray floor + place_z_offset).
void MTCTaskNode::resolveTrayPlacePoint(int cup_index, int num_cups, double& place_x,
                                        double& place_y, double& place_z, double& place_yaw)
{
  place_x = paramd("place_x", -0.31253);
  place_y = paramd("place_y", -0.26902);
  const double tray_z = paramd("place_tray_z", -0.02744);
  // Prefer the live tray detection (x,y only; z stays the tuned claw reference).
  const bool use_tray_det = paramb("use_tray_detection", true);
  if (use_tray_det && tray_received_.load())
  {
    std::lock_guard<std::mutex> lk(tray_mutex_);
    RCLCPP_INFO(LOGGER, "[place] using DETECTED tray (%.3f,%.3f) [param was (%.3f,%.3f)]",
                tray_x_, tray_y_, place_x, place_y);
    place_x = tray_x_;
    place_y = tray_y_;
  }
  else if (use_tray_det)
  {
    RCLCPP_WARN(LOGGER, "[place] use_tray_detection set but no /detected_tray/position "
                "received — falling back to param (%.3f,%.3f)", place_x, place_y);
  }
  // Systematic place-error correction (measured: cup lands ~4 cm negative in x/y).
  place_x += paramd("place_x_offset", 0.044);
  place_y += paramd("place_y_offset", 0.0386);
  // Multi-cup: spread each cup into its own tray slot.
  if (num_cups > 1)
  {
    const std::vector<double> slot_ys = node_->has_parameter("place_slot_ys")
        ? node_->get_parameter("place_slot_ys").as_double_array()
        : node_->declare_parameter<std::vector<double>>(
              "place_slot_ys", std::vector<double>{});  // empty => auto-compute
    if (!slot_ys.empty())
    {
      if (cup_index < static_cast<int>(slot_ys.size()))
      {
        place_y = slot_ys[cup_index];
        RCLCPP_INFO(LOGGER, "[place] cup %d/%d -> explicit y-slot %.3f (x=%.3f)",
                    cup_index + 1, num_cups, place_y, place_x);
      }
      else
        RCLCPP_WARN(LOGGER, "[place] cup_index %d beyond place_slot_ys (%zu)",
                    cup_index, slot_ys.size());
    }
    else
    {
      // Auto-spread evenly along the tray long axis, inset by tray_edge_margin.
      const double tray_len = paramd("tray_length", 0.26);
      const double edge_margin = paramd("tray_edge_margin", 0.06);
      const double long_yaw = paramd("tray_long_axis_yaw", M_PI / 2.0);
      const double usable = std::max(0.0, tray_len - 2.0 * edge_margin);
      const double frac = static_cast<double>(cup_index) / (num_cups - 1) - 0.5;
      const double off  = frac * usable;
      place_x += off * std::cos(long_yaw);
      place_y += off * std::sin(long_yaw);
      RCLCPP_INFO(LOGGER, "[place] cup %d/%d auto-slot off=%.3f m (yaw=%.2f) -> (%.3f,%.3f)",
                  cup_index + 1, num_cups, off, long_yaw, place_x, place_y);
    }
  }
  place_yaw = std::atan2(place_y, place_x) + s_yaw_bias_;
  place_z = tray_z + paramd("place_z_offset", 0.045);
}

// ── Pick the cup, carry it home, then approach the AprilTag dispenser paddle ──
// Orchestrates the sub-steps above: latch the tag (top_cam, valid regardless of arm
// pose), pick+lift, carry the cup LEVEL to home (SRDF "home" rolls the wrist and
// would tip it), reach the tag z, lean/press into the paddle to "fill", retreat,
// and place in the tray. Every carry/lean here is streamLevelMove (straight
// gripper-origin line, per-waypoint position-only IK + level wrist rule) so the cup
// stays upright and the base swings to face the tag without a hand-set Rotation.
void MTCTaskNode::continueToDispenser(int cup_index, int num_cups)
{
  RCLCPP_INFO(LOGGER, "==== [7] Pick + approach AprilTag dispenser (cup %d/%d) ====",
              cup_index + 1, num_cups);

  // Latch the tag world pose UP FRONT (before any motion that could occlude it).
  geometry_msgs::msg::PoseStamped tag;
  if (!latchTagPose(tag)) return;
  const Eigen::Vector3d tag_pos(tag.pose.position.x, tag.pose.position.y, tag.pose.position.z);

  // Radial approach geometry. bearing = base heading to the tag (+ dispenser_bearing_bias
  // to counter position-only IK's base under-rotation). standoff = pre-fill hold-back;
  // fill_depth shifts the fill point (<0 stops short of the tag, >0 presses past).
  const double bearing_bias = paramd("dispenser_bearing_bias", 0.0);
  const double bearing = std::atan2(tag_pos.y(), tag_pos.x()) + bearing_bias;
  const Eigen::Vector3d radial(std::cos(bearing), std::sin(bearing), 0.0);
  const double standoff   = paramd("dispenser_standoff", 0.10);
  const double fill_depth = paramd("dispenser_fill_depth", -0.08);
  const Eigen::Vector3d claw_standoff = tag_pos - standoff   * radial;  // held back
  const Eigen::Vector3d claw_fill     = tag_pos + fill_depth * radial;  // pressed in
  const double tcp_fwd  = paramd("grasp_tcp_forward", 0.05);
  const double tcp_z    = paramd("grasp_tcp_z", 0.0);
  const double face_yaw = bearing + s_yaw_bias_;   // jaw-capture facing (grasp convention)
  RCLCPP_INFO(LOGGER,
      "[dispenser] tag=(%.3f,%.3f,%.3f) bearing=%.3f rad (bias %.3f) | standoff=(%.3f,%.3f,%.3f) fill=(%.3f,%.3f,%.3f)",
      tag_pos.x(), tag_pos.y(), tag_pos.z(), bearing, bearing_bias,
      claw_standoff.x(), claw_standoff.y(), claw_standoff.z(),
      claw_fill.x(), claw_fill.y(), claw_fill.z());

  // [7a] Pick: close, attach, lift.
  if (!runPickLiftTask(paramd("lift_height", 0.12))) return;

  // Ensure the in-process model is loaded (streamLevelMove / FK below need it).
  if (!robot_model_)
  {
    rml_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = rml_->getModel();
  }
  // Neutral all-zero home gripper position (carry pose; the arm cam faces the tag).
  Eigen::Vector3d home_g(0.0, 0.0, 0.0);
  if (robot_model_)
  {
    moveit::core::RobotState hs(robot_model_);
    hs.setToDefaultValues();
    hs.setVariablePositions({ { "Rotation", 0.0 }, { "Pitch", 0.0 }, { "Elbow", 0.0 },
                              { "Wrist_Pitch", 0.0 }, { "Wrist_Roll", 0.0 } });
    hs.update();
    home_g = hs.getGlobalLinkTransform("gripper").translation();
  }

  // FK-based arrival wait (not fixed-time), so the cup's weight-lag can't start the
  // next step before the arm physically settles.
  auto wait_until_at = [&](const Eigen::Vector3d& target, double tmo) {
    if (!robot_model_) return;
    const double tol = paramd("dispenser_home_arrive_tol", 0.02);
    rclcpp::Rate r(10);
    const auto t0 = node_->now();
    while (rclcpp::ok() && (node_->now() - t0).seconds() < tmo)
    {
      sensor_msgs::msg::JointState::SharedPtr js;
      { std::lock_guard<std::mutex> lk(js_mutex_); js = current_js_; }
      if (js && !js->name.empty())
      {
        moveit::core::RobotState rs = seedState(robot_model_, *js);
        const double d = (rs.getGlobalLinkTransform("gripper").translation() - target).norm();
        if (d < tol) { RCLCPP_INFO(LOGGER, "[dispenser] reached target (%.3f m)", d); return; }
      }
      r.sleep();
    }
    RCLCPP_WARN(LOGGER, "[dispenser] arrival wait timed out after %.1fs", tmo);
  };
  auto wait_dur = [](double dur) {
    rclcpp::Rate r(20);
    for (double t = 0.0; t < dur + 0.5 && rclcpp::ok(); t += 0.05) r.sleep();
  };
  auto claw_to_gripper = [&](const Eigen::Vector3d& claw, double yaw) {
    double gx, gy, gz;
    clawMidpointToGripperTarget(claw.x(), claw.y(), claw.z(), yaw,
                                tcp_fwd, tcp_z, gx, gy, gz, s_tcp_lat_);
    return Eigen::Vector3d(gx, gy, gz);
  };

  // [7b] Carry the cup to home (level, gripper closed).
  RCLCPP_INFO(LOGGER, "==== [7b] Move home with cup (gripper closed) ====");
  s_level_seed_rotation_ = std::numeric_limits<double>::quiet_NaN();  // home is Rotation~0
  { const double dur = streamLevelMove(home_g);
    if (dur > 0.0) wait_until_at(home_g, dur + 8.0); }

  // [7b3] Raise the cup to the tag z height (horizontal), so the lean is a pure
  // radial in/out. Pin the base to the side-grasp facing branch so it continues in.
  RCLCPP_INFO(LOGGER, "==== [7b3] Reach AprilTag z=%.3f (horizontal, IK) ====", tag_pos.z());
  s_level_seed_rotation_ = face_yaw + M_PI / 2.0;
  { const Eigen::Vector3d reach_g(home_g.x(), home_g.y(), tag_pos.z());
    const double dur = streamLevelMove(reach_g);
    if (dur > 0.0) wait_until_at(reach_g, dur + 8.0); }

  // [8] Lean the cup to the standoff, then slow-press into the paddle to "fill".
  {
    RCLCPP_INFO(LOGGER, "==== [8] Lean cup into dispenser (level, straight) ====");
    s_level_seed_rotation_ = face_yaw + M_PI / 2.0;
    double dur = streamLevelMove(claw_to_gripper(claw_standoff, face_yaw));
    if (dur <= 0.0) { RCLCPP_ERROR(LOGGER, "[dispenser] standoff stream failed"); return; }
    wait_dur(dur);
    // Press SLOWLY: drop the cartesian speed just for the lean-in, restore after.
    const double fill_speed = paramd("dispenser_fill_speed", 0.04);
    const double saved_cart_speed = s_img_cart_speed_;
    s_img_cart_speed_ = fill_speed;
    RCLCPP_INFO(LOGGER, "==== [8] Pressing paddle at %.3f m/s (was %.3f) ====",
                fill_speed, saved_cart_speed);
    dur = streamLevelMove(claw_to_gripper(claw_fill, face_yaw));
    s_img_cart_speed_ = saved_cart_speed;
    if (dur <= 0.0) { RCLCPP_ERROR(LOGGER, "[dispenser] fill stream failed"); return; }
    wait_dur(dur);
    const double dwell = paramd("dispenser_fill_dwell", 1.5);
    RCLCPP_INFO(LOGGER, "==== [8] Filling (dwell %.1f s) ====", dwell);
    wait_dur(dwell);
  }

  // [9] Retreat to home (level, cup upright).
  RCLCPP_INFO(LOGGER, "==== [9] Retreat to home (level) ====");
  s_level_seed_rotation_ = std::numeric_limits<double>::quiet_NaN();  // home is Rotation~0
  { const double dur = streamLevelMove(home_g);
    if (dur <= 0.0) { RCLCPP_ERROR(LOGGER, "[dispenser] retreat-to-home stream failed"); return; }
    wait_dur(dur); }

  // [10] Place the cup in the tray: approach above, lower, re-seat in bounds,
  // release, then lift off and return home (gripper empty).
  {
    double place_x, place_y, place_z, place_yaw;
    resolveTrayPlacePoint(cup_index, num_cups, place_x, place_y, place_z, place_yaw);
    const double approach_h = paramd("place_approach_height", 0.06);

    RCLCPP_INFO(LOGGER, "==== [10] Place cup at (%.3f,%.3f) ====", place_x, place_y);
    s_level_seed_rotation_ = place_yaw + M_PI / 2.0;  // hold the side-grasp facing branch
    double dur = streamLevelMove(
        claw_to_gripper(Eigen::Vector3d(place_x, place_y, place_z + approach_h), place_yaw));
    if (dur <= 0.0) { RCLCPP_ERROR(LOGGER, "[dispenser] place-approach stream failed"); return; }
    wait_dur(dur);
    dur = streamLevelMove(claw_to_gripper(Eigen::Vector3d(place_x, place_y, place_z), place_yaw));
    if (dur <= 0.0) { RCLCPP_ERROR(LOGGER, "[dispenser] place-lower stream failed"); return; }
    wait_dur(dur);

    reseatArmInBounds();            // keep CurrentState inside URDF bounds for the release
    if (!runReleaseTask()) return;  // open + detach

    // Lift straight off the placed cup, then return to home.
    dur = streamLevelMove(
        claw_to_gripper(Eigen::Vector3d(place_x, place_y, place_z + approach_h), place_yaw));
    if (dur > 0.0) wait_dur(dur);
    s_level_seed_rotation_ = std::numeric_limits<double>::quiet_NaN();  // home is Rotation~0
    dur = streamLevelMove(home_g);
    if (dur > 0.0) wait_dur(dur);
    RCLCPP_INFO(LOGGER, "==== [10] Cup placed — dispenser cycle complete ====");
  }

  // End-of-cycle: restore the overhead cup camera + top_cam tag camera for next cycle.
  {
    const std::string idle_cam = params("servo_idle_camera", "top_cam");
    setActiveCamera(idle_cam);
    setAprilTagCamera("top_cam");
  }
}
// ── Eye-in-hand: switch the perception node's source camera at runtime ───────
// Async (fire-and-forget with a result log). If perception isn't up, warn and
// continue — the servo still runs on whatever camera is currently active.
void MTCTaskNode::setActiveCamera(const std::string& cam)
{
  if (!perception_param_client_->service_is_ready() &&
      !perception_param_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(LOGGER,
                "[servo] perception param service not ready — cannot switch to '%s' "
                "(is perception_node running? check `perception_node_name`)", cam.c_str());
    return;
  }
  perception_param_client_->set_parameters(
      { rclcpp::Parameter("active_camera", cam) },
      [cam](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> fut) {
        const auto results = fut.get();
        if (!results.empty() && results.front().successful)
          RCLCPP_INFO(LOGGER, "[servo] perception active_camera set to '%s'", cam.c_str());
        else
          RCLCPP_WARN(LOGGER, "[servo] perception rejected active_camera='%s' (%s)",
                      cam.c_str(),
                      results.empty() ? "no result" : results.front().reason.c_str());
      });
  RCLCPP_INFO(LOGGER, "[servo] requested perception active_camera='%s'", cam.c_str());
}

// ── Switch the apriltag node's source camera at runtime ──────────────────────
void MTCTaskNode::setAprilTagCamera(const std::string& cam)
{
  if (!apriltag_param_client_->service_is_ready() &&
      !apriltag_param_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(LOGGER,
                "[dispenser] apriltag param service not ready — cannot switch to '%s' "
                "(is apriltag_node running? check `apriltag_node_name`)", cam.c_str());
    return;
  }
  apriltag_param_client_->set_parameters(
      { rclcpp::Parameter("active_camera", cam) },
      [cam](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> fut) {
        const auto results = fut.get();
        if (!results.empty() && results.front().successful)
          RCLCPP_INFO(LOGGER, "[dispenser] apriltag active_camera set to '%s'", cam.c_str());
        else
          RCLCPP_WARN(LOGGER, "[dispenser] apriltag rejected active_camera='%s' (%s)",
                      cam.c_str(),
                      results.empty() ? "no result" : results.front().reason.c_str());
      });
  RCLCPP_INFO(LOGGER, "[dispenser] requested apriltag active_camera='%s'", cam.c_str());
}

// ── Visual servo: closed-loop level side grasp (interlaced with MTC) ─────────
// Timer-driven + async /compute_ik callback (non-blocking), mirroring
// tracking_node.py — a blocking wait_for on the future does not resolve here.
// Eye-in-hand: switches perception to the hand camera for the duration so the
// servo aligns on what the gripper actually sees, then restores the overhead cam.
void MTCTaskNode::visualServoToGrasp()
{
  auto getp = [&](const std::string& n, double d) { return paramd(n, d); };
  const double rate_hz = getp("servo_rate", 10.0);
  // Pre-grasp gap: extra backoff IN FRONT of the claw midpoint. 0 = drive the
  // jaws right onto the spot; set >0 to stop short (e.g. let MTC close the rest).
  // 0.02 = phase 1 stops ~2 cm short of the detected cup center (gentler, avoids
  // ramming the cup); override with the `servo_standoff` param.
  s_standoff_   = getp("servo_standoff", 0.0);
  s_gain_       = getp("servo_gain", 0.6);
  s_max_step_   = getp("servo_max_step", 0.30);
  s_wrist_roll_ = getp("servo_wrist_roll", -M_PI / 2.0);
  s_settle_tol_ = getp("servo_settle_tol", 0.01);
  s_obj_alpha_  = getp("servo_obj_filter_alpha", 0.5);
  s_max_obj_age_ = getp("servo_max_obj_age", 0.4);
  // Smoothing: stretch the trajectory horizon to ~the real command period and
  // optionally low-pass the joints (see members). Reset per-session state.
  s_cmd_time_      = getp("servo_cmd_time", 0.3);
  s_cmd_horizon_k_ = getp("servo_cmd_horizon_k", 1.2);
  s_cmd_smooth_    = getp("servo_cmd_smooth", 0.5);
  have_last_cmd_   = false;
  cmd_filt_init_   = false;
  { std::lock_guard<std::mutex> lk(obj_mutex_); obj_filt_init_ = false; }  // reset filter
  // Wrist-origin → claw-midpoint offset (calibrate via FK; default ~5cm forward).
  s_tcp_fwd_    = getp("grasp_tcp_forward", 0.05);
  s_tcp_z_      = getp("grasp_tcp_z", 0.0);
  s_tcp_lat_    = getp("grasp_tcp_lateral", 0.0);   // fixed-jaw lateral reference
  s_yaw_bias_   = getp("grasp_yaw_bias", YAW_RIGHT_BIAS);  // jaw-gap capture angle
  s_img_p1_pullback_  = getp("servo_img_p1_pullback", 0.0);  // phase-1 stop-short margin
  s_img_p1_lateral_   = getp("servo_img_p1_lateral", 0.01);   // phase-1 lateral bias (+ = left)
  s_img_p1_yaw_scale_ = getp("servo_img_p1_yaw_scale", 0.4);  // phase-1 direction-bias scale
  s_img_p1_depth_stop_m_  = getp("servo_img_p1_depth_stop_m", 0.001);  // arm_cam range to halt at
  s_img_p1_depth_timeout_ = getp("servo_img_p1_depth_timeout", 0.5);  // max age of a stop-range
  // Grasp at the mug's geometric mid-height, not the detector's z (which carries
  // the green-interior centroid's vertical bias). Override via `servo_grasp_z`.
  s_grasp_z_ = CUP_GRASP_Z;
  s_grasp_z_ = getp("servo_grasp_z", s_grasp_z_);

  // ── Image-based (IBVS) servo gains ────────────────────────────────────────
  auto getb = [&](const std::string& n, bool d) { return paramb(n, d); };
  s_img_p1_depth_stop_ = getb("servo_img_p1_depth_stop", false);  // halt phase-1 on arm_cam range
  s_img_k_yaw_        = getp("servo_img_k_yaw", -0.0015);
  // Phase-0 bearing PID. ki defaults to s_img_k_yaw_ (old proportional gain) so
  // the loop is unchanged until kp/kd are raised. kp damps the rate of error
  // change; kd damps overshoot. All three share the arm-cam sign (negative).
  img_yaw_pid_.ki = getp("servo_img_yaw_ki", s_img_k_yaw_);
  img_yaw_pid_.kp = getp("servo_img_yaw_kp", 0.0);
  img_yaw_pid_.kd = getp("servo_img_yaw_kd", 0.0);
  img_yaw_pid_.max_step = 1e9;   // the existing taper/step-cap below bounds the step
  img_yaw_pid_.reset();
  s_img_k_reach_      = getp("servo_img_k_reach", 0.42);
  s_img_deadband_px_  = getp("servo_img_deadband_px", 26.0);
  s_img_done_px_      = getp("servo_img_done_px", 88.0);
  s_img_stop_on_center_ = getp("servo_img_stop_on_center", 0.0) > 0.5;
  s_img_u_offset_px_  = getp("servo_img_u_offset_px", 0.0);
  s_img_reach_min_    = getp("servo_img_reach_min", 0.08);
  s_img_reach_max_    = getp("servo_img_reach_max", 0.35);
  s_img_yaw_step_     = getp("servo_img_yaw_step", 0.006);
  s_img_slow_px_      = getp("servo_img_slow_px", 150.0);
  s_img_slow_floor_   = getp("servo_img_slow_floor", 0.2);
  s_img_slow_pow_     = getp("servo_img_slow_pow", 2.0);
  s_img_reach_step_   = getp("servo_img_reach_step", 0.04);
  s_img_align_tol_    = getp("servo_img_align_tol", 0.5);
  s_img_approach_tol_ = getp("servo_img_approach_tol", 0.2);
  s_img_align_ticks_  = static_cast<int>(getp("servo_img_align_ticks", 3.0));
  s_img_standback_    = getp("servo_img_standback", 0.07);
  s_img_bearing_range_ = getp("servo_img_bearing_range", 0.8);
  s_img_aw_           = getp("servo_img_aw", 1.0);
  s_img_p1_cartesian_ = getb("servo_img_phase1_cartesian", true);
  s_img_cart_speed_   = getp("servo_img_cart_speed", 0.11);
  s_img_cart_step_    = getp("servo_img_cart_step", 0.01);
  img_p1_issued_      = false;
  img_p1_halted_      = false;
  img_p1_goal_.clear();
  img_settled_ = 0;
  img_phase_   = 0;                  // start in VISUAL alignment
  img_have_world_cap_ = false;       // no trusted world distance yet
  servo_max_err_.store(1e9);         // unknown until first ikDone
  {
    // Seed the image-space integrators from the coarse top_cam world detection;
    // image (pixel) error then refines them. Without a seed, fall back to a sane
    // reach straight ahead. (Scoped block: holds obj_mutex_ for the seed only.)
    std::lock_guard<std::mutex> lk(obj_mutex_);
    if (obj_received_.load())
    {
      const double seed_radius = std::hypot(obj_fx_, obj_fy_);
      img_bearing_ = std::atan2(obj_fy_, obj_fx_);
      // Cap forward travel at the seeded object distance minus the standoff, so
      // the claw midpoint stops AT/short of the object instead of pushing through
      // it (the bbox-size loop has no metric stop on its own).
      img_reach_cap_ = std::max(s_img_reach_min_,
                                std::min(s_img_reach_max_, seed_radius - s_standoff_));
      // Seed reach PULLED BACK behind the cap by the standback, so phase 0 centers
      // from a safe distance and phase 1 has a real gap to approach. Never seed
      // past the cap (would start jammed against the object).
      img_reach_   = std::max(s_img_reach_min_,
                              std::min(img_reach_cap_, img_reach_cap_ - s_img_standback_));
      // The seed cap is a real (top_cam) world distance — trust it so phase 1 can
      // advance even when the eye-in-hand cam never yields a usable world fix.
      img_have_world_cap_ = true;
    }
    else
    {
      img_bearing_ = 0.0;
      img_reach_   = 0.2;
      img_reach_cap_ = s_img_reach_max_;  // no seed → fall back to the soft max
    }
    img_tz_ = s_grasp_z_;
    img_bearing0_ = img_bearing_;
    img_tz0_ = img_tz_;
    { std::lock_guard<std::mutex> lkp(px_mutex_); px_have_ = false; }  // wait for fresh pixels
    RCLCPP_INFO(LOGGER,
                "[servo] IMAGE mode — seed bearing=%.2f rad reach=%.3f m z=%.3f m",
                img_bearing_, img_reach_, img_tz_);
  }

  if (!ik_client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "[servo] /compute_ik unavailable — is move_group up?");
    return;
  }

  // ── Eye-in-hand: source object positions from the hand camera ─────────────
  const bool use_arm_cam = paramb("servo_use_arm_cam", true);
  const std::string arm_cam =
      params("servo_arm_camera", std::string("arm_cam"));
  const std::string idle_cam =
      params("servo_idle_camera", std::string("top_cam"));

  eih_active_ = false;
  if (use_arm_cam)
  {
    setActiveCamera(arm_cam);
    eih_active_ = true;  // freeze the grasp-distance cap at the top_cam seed
    // Let a few hand-camera detections arrive before we start commanding, so the
    // servo eases toward the eye-in-hand estimate rather than the stale top-cam
    // one. (The gross move already aimed the hand camera at the object.)
    rclcpp::Rate warmup(2);
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) warmup.sleep();
  }

  servo_done_.store(false);
  ik_busy_.store(false);
  servo_settled_ = 0;
  servo_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&MTCTaskNode::servoTick, this),
      servo_cb_group_);

  RCLCPP_INFO(LOGGER, "[servo] running — easing to level side grasp ...");
  rclcpp::Rate r(20);
  while (rclcpp::ok() && !servo_done_.load())
    r.sleep();

  if (servo_timer_) { servo_timer_->cancel(); servo_timer_.reset(); }
  eih_active_ = false;
  // Camera reset is DEFERRED to the end of the cycle (after place), so perception
  // stays on the arm cam through the grasp + dispenser tag-centering phase instead
  // of flipping back to the overhead cam right after phase 1.
  if (use_arm_cam)
    RCLCPP_INFO(LOGGER, "[servo] leaving perception on '%s'; '%s' restored at cycle end",
                arm_cam.c_str(), idle_cam.c_str());
  RCLCPP_INFO(LOGGER, "[servo] stopped.");
}

// Image-based servo step (decoupled proportional, à la XLeRobot's follow demo):
//   horizontal pixel error → bearing (base Rotation)
//   vertical   pixel error → grasp height (z)
//   apparent bbox size     → approach reach
// The three are integrators, seeded from the coarse world detection and refined
// purely in image space — so a wrong arm-cam extrinsic doesn't bias the loop
// (there's no extrinsic/depth in it). Returns false to HOLD (stale/no pixels).
bool MTCTaskNode::computeImageTarget(double& ox, double& oy, double& target_z, double& yaw)
{
  double u, v, iw, ih, bh;
  bool have;
  {
    std::lock_guard<std::mutex> lk(px_mutex_);
    u = px_u_; v = px_v_; iw = px_iw_; ih = px_ih_; bh = px_bh_;
    have = px_have_;
  }

  // No staleness gate in image mode: convergence is governed entirely by pixel
  // centering (max_err) + bbox target size below. We chase the last known pixel
  // even if the topic momentarily goes quiet — a brief dropout shouldn't stall
  // the approach. We only HOLD if we've never seen a detection at all.
  if (!have || iw <= 0.0 || ih <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[servo] image mode: no pixel detection yet — holding");
    servo_settled_ = 0; img_settled_ = 0;
    return false;
  }

  // Horizontal pixel error from the SETPOINT. The setpoint is offset from image
  // center (s_img_u_offset_px_, + = right) for the side-grasp gripper offset.
  // dx>0: object is RIGHT of setpoint. We only servo LEFT/RIGHT on the image;
  // vertical (dy) is intentionally NOT used — height comes from the world grasp
  // height below, which stopped the gripper diving under the object center.
  const double dx = u - (iw / 2.0 + s_img_u_offset_px_);
  const double dy = v - ih / 2.0;  // logged only

  const double size_frac = bh / ih;
  img_tz_ = s_grasp_z_;  // height is always position-based (world), not visual

  // ── PHASE 0 — VISUAL left/right alignment (reach FROZEN) ────────────────────
  // Only the bearing moves, from the pixel error: small, deadbanded, STEP-LIMITED
  // yaw correction. Reach is NOT advanced here, so during alignment the gripper
  // never drives forward into the object. The sign of servo_img_k_yaw depends on
  // the camera mount — flip it if the arm drives the object OFF-frame.
  if (img_phase_ == 0)
  {
    // PID on the raw pixel error (so the derivative sees continuous dx), then
    // deadband the OUTPUT near centre. Default gains (ki=s_img_k_yaw_, kp=kd=0)
    // reproduce the old `dyaw = k*dx` proportional law exactly.
    double dyaw = img_yaw_pid_.step(dx);
    if (std::abs(dx) < s_img_deadband_px_) dyaw = 0.0;
    // Taper the max step as the object nears the setpoint: within s_img_slow_px_,
    // scale the cap from full (at the zone edge) down to a floor (near centre), so
    // the base decelerates into the target instead of running at full step until
    // the deadband cuts it off. The curve is (|dx|/slow_px)^slow_pow — slow_pow>1
    // bends it so the turn slows HARDER the closer the object gets to centre (a
    // gentle final creep); slow_pow=1 is the old linear taper. Cap is full outside
    // the zone.
    double step_cap = s_img_yaw_step_;
    if (s_img_slow_px_ > 1e-6 && std::abs(dx) < s_img_slow_px_)
    {
      const double frac = std::pow(std::abs(dx) / s_img_slow_px_, s_img_slow_pow_);
      step_cap *= std::max(s_img_slow_floor_, frac);
    }
    dyaw = std::max(-step_cap, std::min(step_cap, dyaw));

    // ── Anti-windup (back-calculation): scale the integrator step by how far the
    // arm still LAGS the last command (servo_max_err_, computed consistently in
    // ikDone). While the arm is catching up we add little new lead, so the command
    // can't run far ahead and leave the arm coasting through centre when the blob
    // momentarily centres (the integrator "won't stop" overshoot). This only
    // MODULATES the integration rate; it never pins the command to a mismatched
    // reference, so unlike a lead-clamp it CANNOT stall — as the arm catches up
    // (err shrinks) the gain returns to ~1, and at centre the deadband (dyaw=0)
    // stops it cleanly. s_img_aw_ = 0 disables it (plain integrator).
    double w = 1.0;
    if (s_img_aw_ > 0.0 && s_img_align_tol_ > 1e-6)
    {
      const double err = servo_max_err_.load();
      if (err < 1e8)  // 1e9 sentinel = no ikDone yet; don't throttle the first steps
        w = std::max(0.1, std::min(1.0, 1.0 - s_img_aw_ * err / s_img_align_tol_));
    }
    img_bearing_ += w * dyaw;
  }

  // Always refresh the metric grasp-distance cap from the LIVE world fix (used as
  // the approach target in phase 1). Hold the last cap/reach if it's stale.
  double world_dist = -1.0;
  {
    std::lock_guard<std::mutex> lk(obj_mutex_);
    const bool fresh = obj_have_stamp_ &&
                       (node_->now() - obj_stamp_).seconds() <= s_max_obj_age_;
    if (obj_received_.load() && fresh)
      world_dist = std::hypot(obj_fx_, obj_fy_);
  }
  // Refresh the metric grasp-distance cap from the LIVE world fix ONLY when it
  // comes from a trusted source (top_cam / PBVS). During the eye-in-hand servo
  // the live fix is the arm_cam depth+TF estimate, which the IBVS design distrusts
  // and which — if it reads too close — collapses the cap below the current reach
  // so phase 1 stalls. In eih we keep the accurate top_cam seed cap instead.
  if (world_dist > 0.0 && !eih_active_)
  {
    img_reach_cap_ = std::max(s_img_reach_min_,
                              std::min(s_img_reach_max_, world_dist - s_standoff_));
    img_have_world_cap_ = true;  // cap is now a trusted real distance
  }

  // ── PHASE 1 — POSITION straight-in approach (bearing FROZEN) ────────────────
  // Ease the reach toward the metric grasp distance (img_reach_cap_) along the
  // now-fixed bearing, step-limited so it creeps in. We do NOT require a FRESH
  // world fix each tick: depth dropouts are chronic on the eye-in-hand cam, and
  // halting the approach on every dropout strands the arm short of the object.
  // Instead we keep creeping toward the LAST trusted cap (never past it — cap is
  // world_dist−standoff, a hard stop), as long as depth worked at least once.
  if (img_phase_ == 1 && img_have_world_cap_)
  {
    double dreach = s_img_k_reach_ * (img_reach_cap_ - img_reach_);
    dreach = std::max(-s_img_reach_step_, std::min(s_img_reach_step_, dreach));
    img_reach_ += dreach;
  }

  // Clamp integrators (a wrong gain sign just pins at a bound — visible/safe).
  img_bearing_ = std::max(img_bearing0_ - s_img_bearing_range_,
                          std::min(img_bearing0_ + s_img_bearing_range_, img_bearing_));
  img_tz_      = std::max(img_tz0_ - 0.10,     std::min(img_tz0_ + 0.10,     img_tz_));
  // Reach upper bound is the LIVE metric grasp distance once we have a world fix
  // (not just reach_max), so the claw midpoint can never sit past the object — and
  // in phase 0 this pulls reach BACK to the cap if it was seeded/left too close.
  const double reach_hi = img_have_world_cap_
                              ? std::min(s_img_reach_max_, img_reach_cap_)
                              : s_img_reach_max_;
  img_reach_   = std::max(s_img_reach_min_,    std::min(reach_hi,           img_reach_));

  // ── Phase 0 → 1 transition ─────────────────────────────────────────────────
  // Advance to the straight approach only once the object is CENTERED left/right
  // AND the arm has physically caught up to the alignment command (joint max_err
  // from the last ikDone < align_tol). PHASE 1 completion (max_err < approach_tol)
  // is decided in ikDone(), where max_err is computed.
  const bool centered = std::abs(dx) < s_img_done_px_;
  if (img_phase_ == 0)
  {
    if (centered && servo_max_err_.load() < s_img_align_tol_)
    {
      if (s_img_stop_on_center_)
      {
        // Halt the instant the blob is centered (and the arm has settled there) —
        // skip the phase-1 straight-in approach entirely.
        RCLCPP_INFO(LOGGER,
                    "[servo] green blob centered (dx=%.0f px, max_err<%.2f) — "
                    "STOPPING (stop_on_center)", dx, s_img_align_tol_);
        servo_done_.store(true);
      }
      else if (++img_settled_ >= s_img_align_ticks_)
      {
        RCLCPP_INFO(LOGGER,
                    "[servo] aligned (dx=%.0f px centered, max_err<%.2f) — "
                    "approaching straight in", dx, s_img_align_tol_);
        img_phase_ = 1; img_settled_ = 0; servo_settled_ = 0;
      }
    }
    else
      // Leaky, not a hard reset: a single off-center tick from the residual
      // centering jitter decays the count by one instead of zeroing it, so an
      // intermittent-centered pattern (0,1,0,1,1,...) still accrues and commits.
      img_settled_ = std::max(0, img_settled_ - 1);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "[servo] IMG p%d dx=%.0f dy=%.0f size=%.2f -> bearing=%.2f "
                       "reach=%.3f/cap=%.3f z=%.3f [centered %d] (err=%.2f)",
                       img_phase_, dx, dy, size_frac, img_bearing_,
                       img_reach_, img_reach_cap_, img_tz_,
                       centered, servo_max_err_.load());

  ox = img_reach_ * std::cos(img_bearing_);
  oy = img_reach_ * std::sin(img_bearing_);
  target_z = img_tz_;
  // Facing = cup bearing + jaw-gap bias. In phase 1 the bias is scaled down so the
  // straight-in direction points more directly at the cup (servo_img_p1_yaw_scale).
  const double yaw_bias = (img_phase_ == 1) ? s_yaw_bias_ * s_img_p1_yaw_scale_ : s_yaw_bias_;
  yaw = img_bearing_ + yaw_bias;  // jaw-capture offset is in centering
  return true;
}

// Build + publish a straight-line Cartesian approach for phase 1. Samples the
// world line from the current claw radius to the grasp distance (cap) along the
// frozen bearing at the fixed grasp height, solves position-only IK + the level
// wrist rule at each sample (the same recipe as bridge/lift/place), and streams
// them as ONE time-parameterized trajectory so the controller glides straight in.
// Returns false if no waypoint was reachable (caller falls back to the integrator).
bool MTCTaskNode::issuePhase1Cartesian(const sensor_msgs::msg::JointState::SharedPtr& js)
{

  // Lazily load the robot model (in-process IK needs it). Keep the loader as a
  // member so its kinematics-plugin class loader isn't destroyed while the cached
  // model's solver objects still exist (avoids the class_loader unload warning).
  if (!robot_model_)
  {
    rml_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = rml_->getModel();
    if (!robot_model_)
    {
      RCLCPP_WARN(LOGGER, "[servo] phase1 cartesian: no robot model — falling back");
      return false;
    }
  }

  const auto* jmg = robot_model_->getJointModelGroup("arm_group");
  moveit::core::RobotState rs = seedState(robot_model_, *js);

  // STRAIGHT-IN, NO SWERVE. The SO-101 has ONE moving jaw, so the object must land
  // IN THE JAW GAP — that offset now lives in the PHASE-0 centering setpoint
  // (servo_img_u_offset_px aims the cup right-of-centre), NOT in a phase-1 swerve.
  // So phase 1 simply closes the gap from the gripper's CURRENT position straight
  // to the detected cup along a Cartesian line, holding the facing at the
  // jaw-capture yaw (= bearing + s_yaw_bias_) the whole way — no rotation along the
  // path means no transition jerk and the travel actually points at the cup.
  // Direction bias scaled down in phase 1 so the straight-in travel points more
  // directly at the cup (full s_yaw_bias_ would head in ~29° off the bearing).
  const double yaw_base = img_bearing_ + s_yaw_bias_ * s_img_p1_yaw_scale_;  // jaw-capture facing (held constant)
  const double z     = img_tz_;
  const double tcp_fwd = s_tcp_fwd_ + s_standoff_;
  // Start = current claw midpoint, recovered from FK by inverting the TCP offset at
  // the current facing (so we truly start where the gripper IS, not a polar guess).
  const Eigen::Vector3d g_now = rs.getGlobalLinkTransform("gripper").translation();
  const double sx = g_now.x() + tcp_fwd * std::cos(yaw_base) + s_tcp_lat_ * std::sin(yaw_base);
  const double sy = g_now.y() + tcp_fwd * std::sin(yaw_base) - s_tcp_lat_ * std::cos(yaw_base);
  // End = cup grasp claw midpoint (detected bearing), pulled back by the phase-1
  // stop-short margin so the approach travels less toward the cup (avoids
  // overshooting / nudging it). Pull-back is along the radial (toward the base).
  const double r_end = std::max(s_img_reach_min_, img_reach_cap_ - s_img_p1_pullback_);
  // Lateral bias: shift the target perpendicular to the bearing. The CCW-perpendicular
  // of heading img_bearing_ is (-sin, cos) — i.e. LEFT of the outward approach — so
  // +s_img_p1_lateral_ nudges the grasp left, - nudges it right.
  const double ex = r_end * std::cos(img_bearing_) - s_img_p1_lateral_ * std::sin(img_bearing_);
  const double ey = r_end * std::sin(img_bearing_) + s_img_p1_lateral_ * std::cos(img_bearing_);
  const double dxc = ex - sx, dyc = ey - sy;
  const double span = std::hypot(dxc, dyc);  // straight-line distance from here to the (pulled-back) target

  // Already at/past the grasp distance — nothing to approach. Seed the arrival
  // test with the current joints so the tracker declares done immediately.
  if (span <= 1e-3)
  {
    img_p1_goal_.clear();
    for (size_t i = 0; i < js->name.size(); ++i) img_p1_goal_[js->name[i]] = js->position[i];
    RCLCPP_INFO(LOGGER, "[servo] phase1 cartesian: already at grasp distance (span=%.3f)", span);
    return true;
  }

  const int N = std::max(1, static_cast<int>(std::ceil(span / s_img_cart_step_)));
  const double seg = span / N;

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = ARM_JOINTS;
  std::map<std::string, double> last_good;
  double cum_t = 0.0;
  int good = 0;

  for (int k = 1; k <= N; ++k)
  {
    // Straight-line interpolation from the current claw midpoint toward the cup,
    // facing held at yaw_base (jaw-capture) — no rotation along the path = no swerve.
    const double f  = static_cast<double>(k) / N;
    const double cx = sx + f * dxc;
    const double cy = sy + f * dyc;
    const double d  = f * span;          // cumulative distance (for the IK-fail log)
    double gx, gy, gz;
    clawMidpointToGripperTarget(cx, cy, z, yaw_base, tcp_fwd, s_tcp_z_, gx, gy, gz, s_tcp_lat_);

    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(gx, gy, gz);
    if (!rs.setFromIK(jmg, target, "gripper", 0.05))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[servo] phase1 cartesian: IK failed at d=%.3f m — skipping waypoint", d);
      continue;  // skip the unreachable sample; keep the reachable ones
    }

    const double R = rs.getVariablePosition("Rotation");
    const double P = rs.getVariablePosition("Pitch");
    const double E = rs.getVariablePosition("Elbow");
    const double WP = levelWristPitch(P, E);                 // level side-grasp wrist rule
    const double WR = s_wrist_roll_;
    // Re-impose the wrist rule on the state so the next waypoint's IK seeds from a
    // continuous, level pose (avoids branch flips along the path).
    rs.setVariablePosition("Wrist_Pitch", WP);
    rs.setVariablePosition("Wrist_Roll", WR);
    rs.update();

    std::map<std::string, double> pj = levelGoal(R, P, E, WR);
    cum_t += std::abs(seg) / std::max(1e-3, s_img_cart_speed_);
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (const auto& j : ARM_JOINTS) pt.positions.push_back(pj[j]);
    pt.time_from_start = rclcpp::Duration::from_seconds(cum_t);
    traj.points.push_back(pt);
    last_good = pj;
    ++good;
  }

  if (good < 1)
  {
    RCLCPP_WARN(LOGGER, "[servo] phase1 cartesian: no reachable waypoints — falling back");
    return false;
  }

  cmd_pub_->publish(traj);
  img_p1_goal_ = last_good;
  RCLCPP_INFO(LOGGER,
              "[servo] phase1 CARTESIAN straight-in to cup, facing yaw %.2f rad: "
              "%d pts, %.3f m, %.2f s @ %.3f m/s",
              yaw_base, good, span, cum_t, s_img_cart_speed_);
  return true;
}

// Stream a straight-line, LEVEL gripper move to `target_g` (world gripper-origin
// position). Mirrors issuePhase1Cartesian: sample the line, position-only IK per
// waypoint, re-impose the level side-grasp wrist rule (Wrist_Pitch=-(Pitch+Elbow),
// Wrist_Roll=-pi/2) so a carried cup stays upright. Returns the trajectory
// duration (s), or 0.0 on failure.
double MTCTaskNode::streamLevelMove(const Eigen::Vector3d& target_g)
{

  if (!robot_model_)
  {
    rml_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = rml_->getModel();
    if (!robot_model_) { RCLCPP_ERROR(LOGGER, "[level-move] no robot model"); return 0.0; }
  }
  const auto* jmg = robot_model_->getJointModelGroup("arm_group");

  sensor_msgs::msg::JointState::SharedPtr js;
  { std::lock_guard<std::mutex> lk(js_mutex_); js = current_js_; }
  if (!js || js->name.empty()) { RCLCPP_ERROR(LOGGER, "[level-move] no joint state"); return 0.0; }

  moveit::core::RobotState rs = seedState(robot_model_, *js);

  const Eigen::Vector3d start_g = rs.getGlobalLinkTransform("gripper").translation();
  const Eigen::Vector3d delta = target_g - start_g;
  const double dist = delta.norm();
  const double step  = s_img_cart_step_  > 1e-4 ? s_img_cart_step_  : 0.01;
  const double speed = s_img_cart_speed_ > 1e-4 ? s_img_cart_speed_ : 0.03;
  const int N = std::max(1, static_cast<int>(std::ceil(dist / step)));

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = ARM_JOINTS;
  double cum_t = 0.0;
  int good = 0;
  for (int k = 1; k <= N; ++k)
  {
    const Eigen::Vector3d g = start_g + delta * (static_cast<double>(k) / N);
    // Pin the base Rotation to the caller's desired side-grasp facing branch
    // (s_level_seed_rotation_ = gripper facing yaw + pi/2). Without this, position-
    // only IK drifts Rotation toward 0 (arm leans sideways) so the carried cup stops
    // pointing at the target. Seeding to the correct branch — which is near the
    // pick pose — keeps IK in it, so the gripper trajectory heads at the AprilTag.
    if (!std::isnan(s_level_seed_rotation_))
    {
      rs.setVariablePosition("Rotation", s_level_seed_rotation_);
      rs.update();
    }
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = g;
    if (!rs.setFromIK(jmg, tf, "gripper", 0.05))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[level-move] IK failed at k=%d/%d — skipping", k, N);
      continue;
    }
    const double R = rs.getVariablePosition("Rotation");
    const double P = rs.getVariablePosition("Pitch");
    const double E = rs.getVariablePosition("Elbow");
    const double WP = levelWristPitch(P, E);
    const double WR = s_wrist_roll_;
    rs.setVariablePosition("Wrist_Pitch", WP);
    rs.setVariablePosition("Wrist_Roll", WR);
    rs.update();

    cum_t += (dist / N) / std::max(1e-3, speed);
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    std::map<std::string, double> pj = levelGoal(R, P, E, WR);
    for (const auto& j : ARM_JOINTS) pt.positions.push_back(pj[j]);
    pt.time_from_start = rclcpp::Duration::from_seconds(cum_t);
    traj.points.push_back(pt);
    ++good;
  }

  if (good < 1) { RCLCPP_ERROR(LOGGER, "[level-move] no reachable waypoints"); return 0.0; }
  cmd_pub_->publish(traj);
  // Diagnostic: show WHERE the base is being commanded. start/target are gripper-
  // origin world points; the bearings are their azimuths about the base; R0/Rn are
  // the first/last commanded Rotation (rad). If Rn's sign/quadrant doesn't match
  // the target bearing, position-only IK chose a wrong base branch (arm swinging
  // the wrong way) rather than a perception error.
  const double start_brg  = std::atan2(start_g.y(), start_g.x());
  const double target_brg = std::atan2(target_g.y(), target_g.x());
  const double R0 = traj.points.front().positions[0];
  const double Rn = traj.points.back().positions[0];
  RCLCPP_INFO(LOGGER,
              "[level-move] streamed %d pts, %.3f m, %.2f s @ %.3f m/s | "
              "start g=(%.3f,%.3f,%.3f) brg=%.2f -> target g=(%.3f,%.3f,%.3f) brg=%.2f | "
              "Rotation %.2f -> %.2f rad",
              good, dist, cum_t, speed,
              start_g.x(), start_g.y(), start_g.z(), start_brg,
              target_g.x(), target_g.y(), target_g.z(), target_brg, R0, Rn);
  return cum_t;
}

// One control cycle: fire a single in-flight position-only IK request.
void MTCTaskNode::servoTick()
{
  if (ik_busy_.exchange(true))
    return;  // a request is still in flight

  sensor_msgs::msg::JointState::SharedPtr js;
  { std::lock_guard<std::mutex> lk(js_mutex_); js = current_js_; }
  if (!js || js->name.empty())
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[servo] waiting for /joint_states ...");
    ik_busy_.store(false);
    return;
  }

  // The claw-midpoint world target + facing yaw for this tick, from arm-cam
  // pixel error (IBVS).
  double ox, oy, target_z, yaw;

  if (!computeImageTarget(ox, oy, target_z, yaw))
  {
    ik_busy_.store(false);  // stale/no pixels — HOLD (computeImageTarget logged)
    return;
  }

  // ── PHASE 1 — streamed Cartesian approach ────────────────────────────────
  // Once aligned, replace the per-tick async IK with ONE straight-line approach
  // trajectory (issued first phase-1 tick), then just track it to completion.
  if (s_img_p1_cartesian_ && img_phase_ == 1)
  {
    // ── Depth-stop (closed-loop on the arm_cam range) ──────────────────────
    // Halt the straight-in approach the instant the live perceived range to the
    // cup drops to/below the threshold. This overrides the open-loop joint-
    // arrival test below, so a too-optimistic reach estimate can't drive the
    // gripper past the cup: we stop at the truly-perceived standoff. If depth is
    // stale/absent the gate does nothing and the joint-arrival fallback stands.
    if (s_img_p1_depth_stop_ && !img_p1_halted_)
    {
      double depth = 0.0; bool fresh = false;
      {
        std::lock_guard<std::mutex> lk(depth_mutex_);
        if (arm_depth_have_)
        {
          depth = arm_depth_;
          fresh = (node_->now() - depth_stamp_).seconds() < s_img_p1_depth_timeout_;
        }
      }
      // Always log what the monitor sees, so the threshold can be read off live
      // (and a wrong/tiny perceived range is obvious even when it doesn't fire).
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "[servo] phase1 depth-stop watch: range=%.4f m fresh=%d "
                           "(stop if <= %.4f)", depth, fresh ? 1 : 0, s_img_p1_depth_stop_m_);
      if (fresh && depth > 1e-3 && depth <= s_img_p1_depth_stop_m_)
      {
        // Stop where we are: command the current arm joints with a short horizon
        // so the controller decelerates and holds (cancels the in-flight glide).
        std::map<std::string, double> cur;
        for (size_t i = 0; i < js->name.size(); ++i) cur[js->name[i]] = js->position[i];
        trajectory_msgs::msg::JointTrajectory stop;
        stop.joint_names = ARM_JOINTS;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        for (const auto& j : ARM_JOINTS) pt.positions.push_back(cur.count(j) ? cur[j] : 0.0);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.1);
        stop.points.push_back(pt);
        cmd_pub_->publish(stop);
        img_p1_halted_ = true;
        RCLCPP_INFO(LOGGER,
                    "[servo] phase1 DEPTH-STOP: perceived range %.3f m <= %.3f m — "
                    "halting approach", depth, s_img_p1_depth_stop_m_);
        servo_done_.store(true);
        ik_busy_.store(false);
        return;
      }
    }

    if (!img_p1_issued_)
    {
      if (issuePhase1Cartesian(js))
        img_p1_issued_ = true;
      // else: IK couldn't build the path — fall through to the per-tick async
      // IK below this tick (the integrator fallback) so the approach still runs.
    }
    if (img_p1_issued_)
    {
      // Track arrival against the final commanded joints. The whole approach is
      // already in flight on the controller; we only watch for it to finish.
      std::map<std::string, double> cur;
      for (size_t i = 0; i < js->name.size(); ++i) cur[js->name[i]] = js->position[i];
      double max_err = 0.0;
      for (const auto& g : img_p1_goal_)
        if (cur.count(g.first))
          max_err = std::max(max_err, std::abs(g.second - cur[g.first]));
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[servo] phase1 CARTESIAN tracking: max_err=%.3f settled=%d/5",
                           max_err, servo_settled_);
      if (max_err < s_img_approach_tol_)
      {
        if (++servo_settled_ >= 5)
        {
          RCLCPP_INFO(LOGGER, "[servo] approach done (cartesian, max_err=%.3f rad)", max_err);
          servo_done_.store(true);
        }
      }
      else
        servo_settled_ = 0;
      ik_busy_.store(false);
      return;
    }
  }

  // Target the CLAW MIDPOINT onto the detected spot (ox, oy, target_z), then back
  // out to the gripper origin IK actually places. The TCP forward offset plus any
  // pre-grasp gap is how far the gripper origin sits behind the spot.
  double gx, gy, gz;
  clawMidpointToGripperTarget(ox, oy, target_z, yaw,
                              s_tcp_fwd_ + s_standoff_, s_tcp_z_, gx, gy, gz, s_tcp_lat_);

  auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  req->ik_request.group_name = "arm_group";
  req->ik_request.ik_link_name = "gripper";
  req->ik_request.pose_stamped.header.frame_id = "world";
  req->ik_request.pose_stamped.pose.position.x = gx;
  req->ik_request.pose_stamped.pose.position.y = gy;
  req->ik_request.pose_stamped.pose.position.z = gz;
  req->ik_request.pose_stamped.pose.orientation.w = 1.0;  // ignored (position-only IK)
  req->ik_request.avoid_collisions = false;               // jaw near object/table else rejected
  req->ik_request.robot_state.joint_state = *js;          // seed from current state
  req->ik_request.timeout = rclcpp::Duration::from_seconds(0.05);

  ik_client_->async_send_request(
      req, [this, js](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture fut) {
        ikDone(fut, js);
      });
}

// IK response: apply the level side-grasp recipe and ease the controller in.
void MTCTaskNode::ikDone(
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future,
    sensor_msgs::msg::JointState::SharedPtr seed)
{

  auto res = future.get();
  ik_busy_.store(false);

  if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[servo] IK: no solution (code %d)", res->error_code.val);
    return;
  }

  std::map<std::string, double> sol, cur;
  const auto& sj = res->solution.joint_state;
  for (size_t i = 0; i < sj.name.size(); ++i) sol[sj.name[i]] = sj.position[i];
  for (size_t i = 0; i < seed->name.size(); ++i) cur[seed->name[i]] = seed->position[i];
  if (!sol.count("Rotation") || !sol.count("Pitch") || !sol.count("Elbow"))
    return;

  // ── Level side-grasp recipe ────────────────────────────────────────────────
  const double R = sol["Rotation"];
  const double P = sol["Pitch"];
  const double E = sol["Elbow"];

  // Ease the positioning joints (Rotation/Pitch/Elbow) and Wrist_Roll toward
  // their targets, then tie Wrist_Pitch to the COMMANDED Pitch+Elbow so the
  // gripper is horizontal on EVERY tick. Easing Wrist_Pitch independently (as
  // before) let the pitch-chain sum drift during the transient, so the gripper
  // stayed tilted all the way in and only leveled at convergence.
  auto ease = [&](const std::string& j, double target) {
    const double c = cur.count(j) ? cur[j] : 0.0;
    double delta = s_gain_ * (target - c);
    delta = std::max(-s_max_step_, std::min(s_max_step_, delta));
    return c + delta;
  };

  double cmd_R  = ease("Rotation", R);
  double cmd_P  = ease("Pitch", P);
  double cmd_E  = ease("Elbow", E);
  double cmd_WR = ease("Wrist_Roll", s_wrist_roll_);

  // Output low-pass: position-only IK can jump to a different solution branch for
  // a nearby target, which the easing would chase as a hard reversal — the
  // visible vibration. Filter the IK-driven joints, then re-tie Wrist_Pitch to
  // the FILTERED Pitch+Elbow so the gripper stays exactly level. alpha=1 = off.
  if (s_cmd_smooth_ < 1.0)
  {
    auto lp = [&](const std::string& j, double v) {
      if (!cmd_filt_init_ || !cmd_filt_.count(j)) return v;
      return s_cmd_smooth_ * v + (1.0 - s_cmd_smooth_) * cmd_filt_[j];
    };
    cmd_R  = lp("Rotation", cmd_R);
    cmd_P  = lp("Pitch", cmd_P);
    cmd_E  = lp("Elbow", cmd_E);
    cmd_WR = lp("Wrist_Roll", cmd_WR);
    cmd_filt_["Rotation"] = cmd_R; cmd_filt_["Pitch"] = cmd_P;
    cmd_filt_["Elbow"] = cmd_E;    cmd_filt_["Wrist_Roll"] = cmd_WR;
    cmd_filt_init_ = true;
  }
  const double cmd_WP = levelWristPitch(cmd_P, cmd_E);   // horizontal every tick (filtered sum), clamped to limit

  std::map<std::string, double> cmd = {
    { "Rotation", cmd_R }, { "Pitch", cmd_P }, { "Elbow", cmd_E },
    { "Wrist_Pitch", cmd_WP }, { "Wrist_Roll", cmd_WR }
  };

  // Horizon = ~the real inter-command period so the controller glides to the
  // target across the whole gap (arriving with ~zero velocity just as the next
  // reply lands) instead of darting in a fixed 0.2 s and freezing — the stop-go
  // that read as spasming. Floor at s_cmd_time_, cap so a stall can't crawl.
  const rclcpp::Time now = node_->now();
  double horizon = s_cmd_time_;
  double cmd_dt = 0.0;
  if (have_last_cmd_)
  {
    cmd_dt = (now - last_cmd_stamp_).seconds();
    horizon = std::max(s_cmd_time_, std::min(1.5, s_cmd_horizon_k_ * cmd_dt));
  }
  last_cmd_stamp_ = now;
  have_last_cmd_  = true;
  // Diagnostic: actual command cadence vs the horizon we hand the controller. If
  // period >> horizon the arm reaches each point and FREEZES (single-point traj,
  // zero end-velocity) before the next command — the stop-go "spasm". If period
  // is also large the loop is IK-round-trip-bound (the real bottleneck).
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "[servo] cmd period=%.3fs (%.1f Hz)  horizon=%.3fs",
                       cmd_dt, cmd_dt > 1e-6 ? 1.0 / cmd_dt : 0.0, horizon);

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = ARM_JOINTS;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  for (const auto& j : ARM_JOINTS) pt.positions.push_back(cmd[j]);
  pt.time_from_start = rclcpp::Duration::from_seconds(horizon);
  traj.points.push_back(pt);
  cmd_pub_->publish(traj);

  // Convergence on the positioning joints + roll (Wrist_Pitch follows them).
  double max_err = 0.0;
  const std::pair<std::string, double> goals[] = {
    { "Rotation", R }, { "Pitch", P }, { "Elbow", E }, { "Wrist_Roll", s_wrist_roll_ }
  };
  for (const auto& g : goals)
  {
    const double c = cur.count(g.first) ? cur[g.first] : 0.0;
    max_err = std::max(max_err, std::abs(g.second - c));
  }

  // Share max_err with computeImageTarget()'s phase-0→1 gate.
  servo_max_err_.store(max_err);

  // Two-phase machine: the 0→1 (alignment-done) transition is decided in
  // computeImageTarget() (it needs the pixel dx). Here we declare the FINAL done —
  // in PHASE 1 (straight approach), once the arm reaches the grasp distance
  // (joint max_err < approach_tol). PHASE 0 alignment is not completion.
  if (img_phase_ == 1 && max_err < s_img_approach_tol_)
  {
    if (++servo_settled_ >= 5)
    {
      RCLCPP_INFO(LOGGER,
                  "[servo] approach done: at grasp distance (max_err=%.3f rad < %.2f)",
                  max_err, s_img_approach_tol_);
      servo_done_.store(true);
    }
  }
  else
    servo_settled_ = 0;
}

// ── Create the pre-servo "gross move" task ───────────────────────────────────
// Sample (RRTConnect) to a level side-grasp standoff in front of the cup so the
// eye-in-hand camera sees it; the visual servo then closes the gap. The claw is
// opened separately via the gripper controller (openGripper() in doTask) before
// this executes.
//
// The standoff joints come from IN-PROCESS position-only IK (same plugin as the
// servo) PLUS the level side-grasp rule, commanded as a JOINT-SPACE goal. We can't
// use MTC's ComputeIK here: position-only IK leaves the wrist free, so the gripper
// wouldn't rotate to the side-grasp orientation. Computing R/P/E ourselves and
// overriding Wrist_Roll=-pi/2, Wrist_Pitch=-(Pitch+Elbow) presents the side grasp,
// and the correctly-oriented open jaws clear the cup cylinder at this standoff.
mtc::Task MTCTaskNode::createTask()
{
  // Snapshot the object position for this task.
  double obj_x, obj_y;
  { std::lock_guard<std::mutex> lock(obj_mutex_); obj_x = obj_x_; obj_y = obj_y_; }

  mtc::Task task;
  task.stages()->setName("standoff for visual servo");
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "arm_group";
  const auto& hand_group_name = "hand_group";
  const auto& hand_frame      = "gripper";
  task.setProperty("group",    arm_group_name);
  task.setProperty("eef",      "gripper");
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("ompl", "RRTConnectkConfigDefault");

  // ── Stage 1: Current state ────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage));
  }

  // ── Stage 1b: Allow object↔table + hand↔table collisions ──────────────────
  // A cup resting on the table is geometrically touching it, and the gross move
  // brings the open jaws near the tabletop; allow those two pairs so the plan
  // isn't vetoed. Every OTHER link stays collision-checked against the table, so
  // the arm still can't drive itself into it.
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object,table)");
    stage->allowCollisions("object", "table", true);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (hand,table)");
    stage->allowCollisions(
        "table",
        task.getRobotModel()->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry(),
        true);
    task.add(std::move(stage));
  }

  // Stage 2 (open hand) is handled outside MTC — the claw is opened via the
  // gripper controller (openGripper() in doTask) before this task executes.

  // ── Stage 3: Sampling move to the object-relative standoff ─────────────────
  // Present the SAME level side-grasp pose the servo (phase 0/1) uses, so the
  // gripper is HORIZONTAL and FACES the cup at the jaw-capture yaw BEFORE phase 0
  // — no orientation jump when servoing starts. Put the CLAW MIDPOINT at a standoff
  // on the cup bearing, then back out to the gripper origin through the TCP offset
  // at yaw = bearing + grasp_yaw_bias. Height follows the SAME servo_grasp_z param
  // the servo uses, so there's no vertical pop when centering starts.
  const double bridge_standoff = paramd("bridge_standoff", 0.16);
  const double tcp_fwd    = paramd("grasp_tcp_forward", 0.05);
  const double tcp_z      = paramd("grasp_tcp_z", 0.0);
  const double tcp_lat    = paramd("grasp_tcp_lateral", 0.0);
  const double pre_gap    = paramd("servo_standoff", 0.02);
  const double yaw_bias   = paramd("grasp_yaw_bias", YAW_RIGHT_BIAS);
  const double standoff_z = paramd("servo_grasp_z", CUP_GRASP_Z);

  const double bearing  = std::atan2(obj_y, obj_x);                // true cup azimuth
  const double yaw_base = bearing + yaw_bias;                      // jaw-capture facing
  const double obj_r    = std::hypot(obj_x, obj_y);
  const double claw_r   = std::max(0.05, obj_r - bridge_standoff);  // claw midpoint, pulled back
  double gx, gy, gz;
  clawMidpointToGripperTarget(claw_r * std::cos(bearing), claw_r * std::sin(bearing),
                              standoff_z, yaw_base, tcp_fwd + pre_gap, tcp_z,
                              gx, gy, gz, tcp_lat);
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(gx, gy, gz);

  // Seed an in-process RobotState from the live joints, solve position-only IK for
  // the gripper origin, then apply the level side-grasp rule.
  auto robot_model = task.getRobotModel();
  moveit::core::RobotState rs = seedStateFromCurrent(robot_model);
  const auto* jmg = robot_model->getJointModelGroup(arm_group_name);
  if (!rs.setFromIK(jmg, target, hand_frame, 0.1))
    RCLCPP_ERROR(LOGGER,
                 "[bridge] position-only IK failed at standoff (%.3f,%.3f,%.3f) — "
                 "MoveTo will likely fail; increase bridge_standoff or check reach",
                 target.translation().x(), target.translation().y(),
                 target.translation().z());
  const double R = rs.getVariablePosition("Rotation");
  const double P = rs.getVariablePosition("Pitch");
  const double E = rs.getVariablePosition("Elbow");
  RCLCPP_INFO(LOGGER, "[bridge] standoff joints R=%.3f P=%.3f E=%.3f WP=%.3f WR=%.3f",
              R, P, E, -(P + E), -M_PI / 2.0);

  auto stage = std::make_unique<mtc::stages::MoveTo>("move to standoff", sampling_planner);
  stage->setGroup(arm_group_name);
  stage->setGoal(levelGoal(R, P, E, -M_PI / 2.0));
  task.add(std::move(stage));

  return task;
}
// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Setup scene with placeholder position (will be updated in doTask)
  mtc_task_node->setupPlanningScene();

  // Blocks until /detected_object/position is received, then plans and executes
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}