#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_open_close");
namespace mtc = moveit::task_constructor;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("gripper_open_close_node", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.1);
  interpolation_planner->setMaxAccelerationScalingFactor(0.1);

  mtc::Task task;
  task.loadRobotModel(node);

  auto stage_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_current));

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup("hand");
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage_close->setGroup("hand");
  stage_close->setGoal("close");
  task.add(std::move(stage_close));

  try { task.init(); }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    rclcpp::shutdown();
    return 1;
  }

  if (!task.plan(1)) {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    rclcpp::shutdown();
    return 1;
  }

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    RCLCPP_ERROR(LOGGER, "Execution failed");
  else
    RCLCPP_INFO(LOGGER, "Gripper open/close completed");

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}