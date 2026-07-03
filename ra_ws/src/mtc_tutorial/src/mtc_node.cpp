#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
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

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.5, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.17207;
  pose.position.y = -0.15373;
  pose.position.z = 0.2765;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  // Add a fixed platform collider directly beneath the robot base
  // moveit_msgs::msg::CollisionObject platform;
  // platform.id = "platform";
  // platform.header.frame_id = "world";
  // platform.primitives.resize(1);
  // platform.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  // // dimensions: x, y, z (length, width, height)
  // platform.primitives[0].dimensions = { 1.0, 1.0, 0.02 };

  // geometry_msgs::msg::Pose platform_pose;
  // platform_pose.position.x = 0.0;
  // platform_pose.position.y = 0.0;
  // // Place the top surface at z = 0.0 assuming robot base at z=0.0; center is at -height/2
  // platform_pose.position.z = -0.01;
  // platform_pose.orientation.w = 1.0;
  // platform.pose = platform_pose;

  // psi.applyCollisionObject(platform);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.enableIntrospection();
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "==== [1] Planning ====");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "==== [1] Planning SUCCEEDED — %zu solution(s) ====",
              task_.solutions().size());

  RCLCPP_INFO(LOGGER, "==== [2] Inspecting solution ====");
  const auto& solution = *task_.solutions().front();
  RCLCPP_INFO(LOGGER, "Solution cost: %.4f", solution.cost());



  RCLCPP_INFO(LOGGER, "==== [3] Publishing solution to RViz ====");
  task_.introspection().publishSolution(solution);
  RCLCPP_INFO(LOGGER, "==== [3.5] Converting solution to message ====");
  moveit_task_constructor_msgs::msg::Solution sol_msg;
  solution.toMsg(sol_msg, &task_.introspection());
  RCLCPP_INFO(LOGGER, "Sub-trajectories: %zu", sol_msg.sub_trajectory.size());
  for (size_t i = 0; i < sol_msg.sub_trajectory.size(); ++i)
  {
    const auto& jt = sol_msg.sub_trajectory[i].trajectory.joint_trajectory;
    RCLCPP_INFO(LOGGER, "  [%zu] joints=%zu  points=%zu  controllers=%zu",
                i, jt.joint_names.size(), jt.points.size(),
                sol_msg.sub_trajectory[i].execution_info.controller_names.size());
    // print joint names
    std::string names;
    for (const auto& n : jt.joint_names) names += n + " ";
    RCLCPP_INFO(LOGGER, "       joint_names: [%s]", names.c_str());
    // print controllers
    for (const auto& c : sol_msg.sub_trajectory[i].execution_info.controller_names)
      RCLCPP_INFO(LOGGER, "       controller: %s", c.c_str());
    // print first and last waypoint
    if (!jt.points.empty())
    {
      std::string first, last;
      for (double p : jt.points.front().positions) first += std::to_string(p) + " ";
      for (double p : jt.points.back().positions)  last  += std::to_string(p) + " ";
      RCLCPP_INFO(LOGGER, "       first waypoint: [%s]", first.c_str());
      RCLCPP_INFO(LOGGER, "       last  waypoint: [%s]", last.c_str());
    }
  }

  RCLCPP_INFO(LOGGER, "==== [4] Executing ====");
  auto result = task_.execute(solution);
  RCLCPP_INFO(LOGGER, "Execution error code: %d", result.val);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code " << result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "==== [4] Execution SUCCEEDED ====");
  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm_group";
  const auto& hand_group_name = "hand_group";
  const auto& hand_frame = "gripper";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "gripper");
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("ompl", "RRTConnect");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.1);  // slow it down
  interpolation_planner->setMaxAccelerationScalingFactor(0.1);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open_grip");
  current_state_ptr = stage_open_hand.get();
  task.add(std::move(stage_open_hand));

  // ── rotate wrist (minimal test — add stages below one by one) ──────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("rotate wrist", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("pre_grasp");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ── pick ─────────────────────────────────────────────────────────────────────────────
  mtc::Stage* attach_object_stage = nullptr;
  

  // Connect arm to pre-grasp position (arm only, jaw already open)
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{
            { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });


    // Approach object
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object (cartesian)", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.08);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      vec.vector.y = 0.0;
      vec.vector.z = 0.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }



    // Grasp pose IK
    {
      Eigen::Quaterniond home_q(-0.495, -0.495, -0.504, 0.504);
      Eigen::Quaterniond rot_x(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond side_clamp_q = (home_q * rot_x).normalized();

      auto stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setMonitoredStage(current_state_ptr);

      geometry_msgs::msg::PoseStamped grasp_pose;
      grasp_pose.header.frame_id = "object";
      grasp_pose.pose.position.x = -0.07;
      grasp_pose.pose.position.y = 0.04;
      grasp_pose.pose.position.z = 0;
      grasp_pose.pose.orientation.x = side_clamp_q.x();
      grasp_pose.pose.orientation.y = side_clamp_q.y();
      grasp_pose.pose.orientation.z = side_clamp_q.z();
      grasp_pose.pose.orientation.w = side_clamp_q.w();
      stage->setPose(grasp_pose);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // wrapper->properties().set("ignore_collisions", true);
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->setGroup(arm_group_name);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    auto links = task.getRobotModel()
        ->getJointModelGroup(hand_group_name)
        ->getLinkModelNamesWithCollisionGeometry();
    for (const auto& l : links)
        RCLCPP_INFO(LOGGER, "hand link: %s", l.c_str());

    // Allow collision between hand and object so gripper can close
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    // Disable object collisions so gripper can close around it
    // {
    //   auto stage =
    //       std::make_unique<mtc::stages::ModifyPlanningScene>("disable object collisions");
    //   stage->allowCollisions("object", true);
    //   task.add(std::move(stage));  // task.add, not grasp->insert
    // }

    // Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close_grip");
      grasp->insert(std::move(stage));
    }

    // Attach object to gripper
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift object
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.5);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  } // end pick SerialContainer

// ── move to place ────────────────────────────────────────────────────────
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  
  {
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });
      {
        // Sample place pose
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject("object");

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world";
        target_pose_msg.pose.position.x = 0.08263;
        target_pose_msg.pose.position.y = -0.28679;   // shift in Y, confirmed reachable
        target_pose_msg.pose.position.z = 0.26528;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }
      
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open_grip");
        place->insert(std::move(stage));
      }
      
      {
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions("object",
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              false);
        place->insert(std::move(stage));
      }
      
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject("object", hand_frame);
        place->insert(std::move(stage));
      }
      
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.0, 0.05);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "retreat");
        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.x = 0.0;
        vec.vector.y = 0.1;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      task.add(std::move(place));
  }
  
  // ── return home ───────────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }
  
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close_grip");
    task.add(std::move(stage));
  }

  // ── COMMENTED: full place container ───────────────────────────────────
  // Uncomment after pick works.
  //
  // {
  //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
  //       "move to place",
  //       mtc::stages::Connect::GroupPlannerVector{
  //           { arm_group_name, sampling_planner },
  //           { hand_group_name, interpolation_planner } });
  //   stage_move_to_place->setTimeout(5.0);
  //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  //   task.add(std::move(stage_move_to_place));
  // }

  return task;
}

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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}