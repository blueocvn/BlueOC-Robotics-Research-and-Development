// mtc_ik_check.cpp
//
// Directly queries the IK solver for each Cartesian pose without planning or
// executing anything. Prints joint values if IK succeeds, or the failure reason.
//
// Usage (pass pose as ROS params, or test all built-in poses):
//   ros2 launch your_pkg your_launch.py        # tests all built-in poses
//
// Or test a single custom pose:
//   ros2 launch your_pkg your_launch.py \
//     x:=0.26 y:=-0.279 z:=0.263 qx:=0.0 qy:=0.0 qz:=0.0 qw:=1.0

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/msg/pose.hpp>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_ik_check");

// ── one IK attempt ────────────────────────────────────────────────────────────

struct IKResult
{
  bool        success;
  std::string label;
  std::string failure_reason;
  std::vector<double> joint_values;
};

IKResult checkIK(moveit::core::RobotState&              robot_state,
                 const moveit::core::JointModelGroup*   jmg,
                 const std::string&                     tip_link,
                 const std::string&                     label,
                 const Eigen::Isometry3d&               target)
{
  IKResult result;
  result.label = label;

  // Reset to default before each attempt for a clean starting point
  robot_state.setToDefaultValues();

  // Attempt IK with 0.1s timeout, 5 attempts
  bool found = robot_state.setFromIK(jmg, target, tip_link, 0.1,
    [](moveit::core::RobotState*, const moveit::core::JointModelGroup*, const double*) {
      return true;  // accept any collision-free solution
    });

  result.success = found;

  if (found)
  {
    robot_state.copyJointGroupPositions(jmg, result.joint_values);
  }
  else
  {
    // Check if target is within bounds by examining the transform
    Eigen::Vector3d pos = target.translation();
    double dist = pos.norm();
    if (dist > 1.5)
      result.failure_reason = "Pose likely out of reach (distance from base: " +
                              std::to_string(dist) + " m)";
    else
      result.failure_reason = "IK solver returned no solution (collision, singularity, or bad orientation)";
  }

  return result;
}

void printResult(const IKResult& r, const moveit::core::JointModelGroup* jmg)
{
  if (r.success)
  {
    RCLCPP_INFO(LOGGER, "[PASS] %s", r.label.c_str());
    auto names = jmg->getVariableNames();
    for (size_t i = 0; i < names.size(); ++i)
      RCLCPP_INFO(LOGGER, "       %s = %.4f rad", names[i].c_str(), r.joint_values[i]);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "[FAIL] %s", r.label.c_str());
    RCLCPP_ERROR(LOGGER, "       Reason: %s", r.failure_reason.c_str());
  }
}

// ── build Isometry from xyzw ──────────────────────────────────────────────────

Eigen::Isometry3d makePose(double x, double y, double z,
                           double qx, double qy, double qz, double qw)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  t.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return t;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mtc_ik_check", options);

  // Declare custom single-pose params (optional)
  auto decl = [&](const std::string& name, double def) {
    if (!node->has_parameter(name)) node->declare_parameter<double>(name, def);
  };
  decl("x",  0.0); decl("y",  0.0); decl("z",  0.0);
  decl("qx", 0.0); decl("qy", 0.0); decl("qz", 0.0); decl("qw", 1.0);

  bool custom_pose =
      node->get_parameter("x").as_double() != 0.0 ||
      node->get_parameter("y").as_double() != 0.0 ||
      node->get_parameter("z").as_double() != 0.0;

  // ── load robot model ───────────────────────────────────────────────────────
  RCLCPP_INFO(LOGGER, "Loading robot model...");
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  auto robot_model = loader.getModel();

  if (!robot_model)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load robot model. "
                         "Make sure robot_description is on the parameter server.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Robot model loaded: %s", robot_model->getName().c_str());

  const std::string arm_group  = "arm_group";
  const std::string tip_link   = "gripper";   // hand_frame from mtc_node.cpp

  const auto* jmg = robot_model->getJointModelGroup(arm_group);
  if (!jmg)
  {
    RCLCPP_FATAL(LOGGER, "Joint model group '%s' not found. "
                         "Check your SRDF group names.", arm_group.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Print which IK plugin is loaded
  const auto& solver = jmg->getSolverInstance();
  if (solver)
    RCLCPP_INFO(LOGGER, "IK solver: %s  tip: %s",
                solver->getGroupName().c_str(),
                solver->getTipFrame().c_str());
  else
    RCLCPP_WARN(LOGGER, "No IK solver found for group '%s' — "
                        "check kinematics.yaml", arm_group.c_str());

  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  // side_clamp orientation — mirrors mtc_node.cpp exactly
  // home_q kept separate so diagnostic sweep can test it alone
  const Eigen::Quaterniond home_q(-0.495, -0.495, -0.504, 0.504);
  const Eigen::Quaterniond rot_x(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond scq = (home_q * rot_x).normalized();

  int pass = 0, fail = 0;

  // ── custom single pose mode ────────────────────────────────────────────────
  if (custom_pose)
  {
    double x  = node->get_parameter("x").as_double();
    double y  = node->get_parameter("y").as_double();
    double z  = node->get_parameter("z").as_double();
    double qx = node->get_parameter("qx").as_double();
    double qy = node->get_parameter("qy").as_double();
    double qz = node->get_parameter("qz").as_double();
    double qw = node->get_parameter("qw").as_double();

    RCLCPP_INFO(LOGGER,
                "Testing custom pose: xyz=(%.3f, %.3f, %.3f)  qxyzw=(%.3f, %.3f, %.3f, %.3f)",
                x, y, z, qx, qy, qz, qw);

    auto r = checkIK(robot_state, jmg, tip_link, "Custom pose",
                     makePose(x, y, z, qx, qy, qz, qw));
    printResult(r, jmg);
    r.success ? ++pass : ++fail;
  }
  else
  {
    // ── built-in poses from mtc_node.cpp ──────────────────────────────────
    RCLCPP_INFO(LOGGER, "\n========== IK Check: all MTC poses ==========");

    // ── Reach sweep: find where IK starts failing along X ─────────────────
    // Fixes y=-0.279, z=0.263, identity orientation (simplest case).
    // Sweeps X from 0.05 → 0.35 in 3 cm steps to find the reachable boundary.

    const double py = -0.279, pz = 0.263;

    struct PoseEntry { std::string label; Eigen::Isometry3d pose; };
    std::vector<PoseEntry> poses;

    for (double x = 0.05; x <= 0.35; x += 0.03)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "X-sweep x=%.2f  y=%.3f  z=%.3f  identity", x, py, pz);
      poses.push_back({ std::string(buf), makePose(x, py, pz, 0.0, 0.0, 0.0, 1.0) });
    }

    // Also sweep Y at the target X to see if y=-0.279 itself is the issue
    RCLCPP_INFO(LOGGER, "--- X sweep at y=%.3f z=%.3f ---", py, pz);
    for (double y = -0.05; y >= -0.40; y -= 0.05)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "Y-sweep x=0.26  y=%.2f  z=%.3f  identity", y, pz);
      poses.push_back({ std::string(buf), makePose(0.26, y, pz, 0.0, 0.0, 0.0, 1.0) });
    }

    for (auto& entry : poses)
    {
      auto r = checkIK(robot_state, jmg, tip_link, entry.label, entry.pose);
      printResult(r, jmg);
      r.success ? ++pass : ++fail;
      RCLCPP_INFO(LOGGER, ""); // blank line between poses
    }
  }

  RCLCPP_INFO(LOGGER,
              "\n============================\n"
              "  IK SUMMARY\n"
              "  PASS: %d   FAIL: %d   TOTAL: %d\n"
              "============================",
              pass, fail, pass + fail);

  rclcpp::shutdown();
  return (fail == 0) ? 0 : 1;
}