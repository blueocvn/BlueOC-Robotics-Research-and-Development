// mtc_pose_test_node.cpp
//
// Run via your existing launch file. Select what to test with a ROS parameter:
//
//   ros2 launch your_pkg your_launch.py test_mode:=joints
//   ros2 launch your_pkg your_launch.py test_mode:=cartesian
//   ros2 launch your_pkg your_launch.py test_mode:=all        (default)
//
// Between each pose the node prints a prompt and waits for you to publish
// a trigger on /pose_tester/next  (std_msgs/Empty) – or press the confirm
// button in rqt_publisher – because launch files close stdin.
//
//   ros2 topic pub --once /pose_tester/next std_msgs/msg/Empty "{}"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
#include <Eigen/Geometry>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pose_tester");

// ── Step descriptor ───────────────────────────────────────────────────────────

enum class StepKind { NAMED_GOAL, CARTESIAN_POSE };

struct TestStep
{
  StepKind    kind;
  std::string label;

  // For NAMED_GOAL
  std::string group_name;
  std::string goal_name;

  // For CARTESIAN_POSE
  geometry_msgs::msg::PoseStamped target_pose;
};

// ── Node ─────────────────────────────────────────────────────────────────────

class PoseTesterNode : public rclcpp::Node
{
public:
  explicit PoseTesterNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mtc_pose_tester", options)
  {
    if (!this->has_parameter("test_mode"))
      this->declare_parameter<std::string>("test_mode", "all");  // joints | cartesian | all

    next_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/pose_tester/next", 10,
        [this](std_msgs::msg::Empty::SharedPtr /*msg*/) {
          {
            std::lock_guard<std::mutex> lk(cv_mutex_);
            triggered_ = true;
          }
          cv_.notify_one();
        });
  }

  // Block until /pose_tester/next is published
  void waitForTrigger(const std::string& label)
  {
    triggered_ = false;
    RCLCPP_INFO(LOGGER,
                "\n──────────────────────────────────────────\n"
                "  READY  »  %s\n"
                "  Publish to advance:\n"
                "    ros2 topic pub --once /pose_tester/next std_msgs/msg/Empty \"{}\"\n"
                "──────────────────────────────────────────",
                label.c_str());

    std::unique_lock<std::mutex> lk(cv_mutex_);
    cv_.wait(lk, [this] { return triggered_.load(); });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr next_sub_;
  std::atomic<bool>       triggered_{ false };
  std::mutex              cv_mutex_;
  std::condition_variable cv_;

public:
  // Expose cv helpers to runner
  std::mutex&              mutex()   { return cv_mutex_; }
  std::condition_variable& condvar() { return cv_; }
};

// ── Runner ────────────────────────────────────────────────────────────────────

class PoseTesterRunner
{
public:
  PoseTesterRunner(std::shared_ptr<PoseTesterNode> node)
  : node_(node)
  , arm_mgi_ (node, "arm_group")
  , hand_mgi_(node, "hand_group")
  {
    arm_mgi_.setMaxVelocityScalingFactor(0.1);
    arm_mgi_.setMaxAccelerationScalingFactor(0.1);
    hand_mgi_.setMaxVelocityScalingFactor(0.1);
    hand_mgi_.setMaxAccelerationScalingFactor(0.1);
    arm_mgi_.setPoseReferenceFrame("world");
    arm_mgi_.setPlanningTime(10.0);
    arm_mgi_.setNumPlanningAttempts(5);
    arm_mgi_.setPlannerId("RRTConnect");
  }

  void run()
  {
    // Drive to home first so the planner always starts from a known configuration
    RCLCPP_INFO(LOGGER, "Moving to home before tests...");
    arm_mgi_.setNamedTarget("home");
    auto home_result = arm_mgi_.move();
    if (home_result != moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_WARN(LOGGER, "Failed to reach home — continuing anyway");

    const std::string mode =
        node_->get_parameter("test_mode").as_string();

    RCLCPP_INFO(LOGGER, "test_mode = \"%s\"", mode.c_str());

    auto steps = buildSteps(mode);

    if (steps.empty()) {
      RCLCPP_ERROR(LOGGER,
                   "Unknown test_mode \"%s\". Use: joints | cartesian | all",
                   mode.c_str());
      return;
    }

    int pass = 0, fail = 0;

    for (const auto& step : steps)
    {
      node_->waitForTrigger(step.label);
      bool ok = execute(step);
      ok ? ++pass : ++fail;
    }

    // Always return home
    TestStep home_step;
    home_step.kind       = StepKind::NAMED_GOAL;
    home_step.label      = "Return home";
    home_step.group_name = "arm_group";
    home_step.goal_name  = "home";
    node_->waitForTrigger(home_step.label);
    execute(home_step) ? ++pass : ++fail;

    RCLCPP_INFO(LOGGER,
                "\n============================\n"
                "  TEST SUMMARY  (mode: %s)\n"
                "  PASS: %d   FAIL: %d   TOTAL: %d\n"
                "============================",
                mode.c_str(), pass, fail, pass + fail);
  }

private:
  // ── build step list from mode ───────────────────────────────────────────

  std::vector<TestStep> buildSteps(const std::string& mode)
  {
    std::vector<TestStep> steps;

    if (mode == "joints" || mode == "all")
    {
      // Mirrors stage order in mtc_node.cpp
      steps.push_back(makeNamedGoal("Named goal: open_grip",  "hand_group", "open_grip"));
      steps.push_back(makeNamedGoal("Named goal: pre_grasp",  "arm_group",  "pre_grasp"));
      steps.push_back(makeNamedGoal("Named goal: close_grip", "hand_group", "close_grip"));
      steps.push_back(makeNamedGoal("Named goal: home",       "arm_group",  "home"));
    }

    if (mode == "cartesian" || mode == "all")
    {
      // Confirmed reachable by mtc_ik_check sweep (diagonal ~0.283 m, well under ~0.35 m limit)
      steps.push_back(makeCartesian(
          "Cartesian: confirmed target (x=0.20, y=-0.20, z=0.263)",
          0.20, -0.20, 0.263,
          0.0, 0.0, 0.0, 1.0));
    }

    return steps;
  }

  // ── execute a single step ───────────────────────────────────────────────

  bool execute(const TestStep& step)
  {
    bool ok = false;

    if (step.kind == StepKind::NAMED_GOAL)
    {
      auto& mgi = (step.group_name == "arm_group") ? arm_mgi_ : hand_mgi_;
      mgi.setNamedTarget(step.goal_name);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      ok = (mgi.plan(plan)    == moveit::core::MoveItErrorCode::SUCCESS) &&
           (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    else  // CARTESIAN_POSE
    {
      arm_mgi_.setPoseTarget(step.target_pose.pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      ok = (arm_mgi_.plan(plan)    == moveit::core::MoveItErrorCode::SUCCESS) &&
           (arm_mgi_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      arm_mgi_.clearPoseTargets();
    }

    if (ok)
      RCLCPP_INFO(LOGGER,  "[PASS] %s", step.label.c_str());
    else
      RCLCPP_ERROR(LOGGER, "[FAIL] %s", step.label.c_str());

    return ok;
  }

  // ── helpers ─────────────────────────────────────────────────────────────

  static TestStep makeNamedGoal(const std::string& label,
                                const std::string& group,
                                const std::string& goal)
  {
    TestStep s;
    s.kind       = StepKind::NAMED_GOAL;
    s.label      = label;
    s.group_name = group;
    s.goal_name  = goal;
    return s;
  }

  static TestStep makeCartesian(const std::string& label,
                                double x, double y, double z,
                                double qx, double qy, double qz, double qw)
  {
    TestStep s;
    s.kind                         = StepKind::CARTESIAN_POSE;
    s.label                        = label;
    s.target_pose.header.frame_id  = "world";
    s.target_pose.pose.position.x  = x;
    s.target_pose.pose.position.y  = y;
    s.target_pose.pose.position.z  = z;
    s.target_pose.pose.orientation.x = qx;
    s.target_pose.pose.orientation.y = qy;
    s.target_pose.pose.orientation.z = qz;
    s.target_pose.pose.orientation.w = qw;
    return s;
  }

  std::shared_ptr<PoseTesterNode>               node_;
  moveit::planning_interface::MoveGroupInterface arm_mgi_;
  moveit::planning_interface::MoveGroupInterface hand_mgi_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PoseTesterNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  PoseTesterRunner runner(node);
  runner.run();

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}