"""Add Ackermann drive control to the JetRacer and split arm/jetracer TF onto
separate namespaced topics.  Headless.

  * /Graph/JetRacer_Ackermann : ROS2 AckermannDriveStamped on /jetracer/ackermann_cmd
        -> AckermannController -> steer (position) + rear-wheel (velocity) articulation control
  * /Graph/ROS_TF (arm)       : re-scoped to /arm/tf, arm frames only
  * /Graph/ROS_TF_JetRacer    : new, publishes /jetracer/tf  (base_link + lidar + camera)

Run:  cd simulation && /home/apc/isaacsim/python.sh add_ackermann_and_split_tf.py
"""
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Sdf

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

USD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_sim.usd")
ctx = omni.usd.get_context()
ctx.open_stage(USD)
simulation_app.update()
stage = ctx.get_stage()

ROBOT = "/jetracer_docking_scene"
CHASSIS = ROBOT + "/chassis"
LIDAR = CHASSIS + "/RPLidar/RPLidar_S2E"
CAM = ROBOT + "/front_cam_mount/FrontCamera"
keys = og.Controller.Keys

# JetRacer measured geometry
WHEEL_BASE = 0.166      # front-rear axle distance
TRACK_WIDTH = 0.18      # rear track (left-right rear wheels)
WHEEL_R = 0.0325
MAX_STEER = 0.5236      # 30 deg, matches steer joint limits

# --------------------------------------------------------------------------------------
# 1) Ackermann drive graph
# --------------------------------------------------------------------------------------
ACK = "/Graph/JetRacer_Ackermann"
og.Controller.edit(
    {"graph_path": ACK, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("SubscribeAckermann", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
            ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
            ("SteerController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("DriveController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("SubscribeAckermann.inputs:nodeNamespace", "/jetracer"),
            ("SubscribeAckermann.inputs:topicName", "ackermann_cmd"),
            ("AckermannController.inputs:wheelBase", WHEEL_BASE),
            ("AckermannController.inputs:trackWidth", TRACK_WIDTH),
            ("AckermannController.inputs:frontWheelRadius", WHEEL_R),
            ("AckermannController.inputs:backWheelRadius", WHEEL_R),
            ("AckermannController.inputs:maxWheelRotation", MAX_STEER),
            # steering: the two front steer joints, position controlled
            ("SteerController.inputs:targetPrim", ROBOT),
            ("SteerController.inputs:robotPath", ROBOT),
            ("SteerController.inputs:jointNames", ["steer_joint_L", "steer_joint_R"]),
            # drive: all four wheels in FL,FR,RL,RR order (front wheels have no drive gains
            # -> harmlessly ignored; rear wheels are velocity-driven)
            ("DriveController.inputs:targetPrim", ROBOT),
            ("DriveController.inputs:robotPath", ROBOT),
            ("DriveController.inputs:jointNames",
             ["wheel_joint_FL", "wheel_joint_FR", "wheel_joint_RL", "wheel_joint_RR"]),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "SubscribeAckermann.inputs:execIn"),
            ("Context.outputs:context", "SubscribeAckermann.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
            ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:dt"),
            ("SubscribeAckermann.outputs:speed", "AckermannController.inputs:speed"),
            ("SubscribeAckermann.outputs:steeringAngle", "AckermannController.inputs:steeringAngle"),
            ("SubscribeAckermann.outputs:acceleration", "AckermannController.inputs:acceleration"),
            # apply outputs
            ("AckermannController.outputs:execOut", "SteerController.inputs:execIn"),
            ("AckermannController.outputs:execOut", "DriveController.inputs:execIn"),
            ("AckermannController.outputs:wheelAngles", "SteerController.inputs:positionCommand"),
            ("AckermannController.outputs:wheelRotationVelocity", "DriveController.inputs:velocityCommand"),
        ],
    },
)
print("[ackermann] built", ACK)

# --------------------------------------------------------------------------------------
# 2) Re-scope the existing arm TF publisher -> /arm/tf, arm frames only
# --------------------------------------------------------------------------------------
arm_tf = stage.GetPrimAtPath("/Graph/ROS_TF/PublisherTF")
arm_tf.GetAttribute("inputs:nodeNamespace").Set("/arm")
arm_tf.GetAttribute("inputs:topicName").Set("tf")
arm_tf.GetRelationship("inputs:targetPrims").SetTargets(["/so101_new_calib/base"])
print("[tf] arm publisher -> /arm/tf, targets:", arm_tf.GetRelationship("inputs:targetPrims").GetTargets())

# base_link frame name for the jetracer chassis
stage.GetPrimAtPath(CHASSIS).CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String, True).Set("base_link")

# --------------------------------------------------------------------------------------
# 3) New jetracer TF publisher -> /jetracer/tf  (base_link + sensors, rooted at world)
# --------------------------------------------------------------------------------------
JTF = "/Graph/ROS_TF_JetRacer"
og.Controller.edit(
    {"graph_path": JTF, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublisherTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("PublisherTF.inputs:nodeNamespace", "/jetracer"),
            ("PublisherTF.inputs:topicName", "tf"),
            ("PublisherTF.inputs:targetPrims", [CHASSIS, LIDAR, CAM]),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublisherTF.inputs:execIn"),
            ("Context.outputs:context", "PublisherTF.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublisherTF.inputs:timeStamp"),
        ],
    },
)
print("[tf] built", JTF, "-> /jetracer/tf")

# --------------------------------------------------------------------------------------
# 4) verify + save
# --------------------------------------------------------------------------------------
simulation_app.update()
checks = {
    "ackermann graph": ACK,
    "ackermann subscriber": ACK + "/SubscribeAckermann",
    "ackermann controller": ACK + "/AckermannController",
    "steer controller": ACK + "/SteerController",
    "drive controller": ACK + "/DriveController",
    "jetracer tf graph": JTF,
    "jetracer tf publisher": JTF + "/PublisherTF",
}
ok = True
for label, path in checks.items():
    v = stage.GetPrimAtPath(path).IsValid()
    ok = ok and v
    print(f"  [{'OK' if v else 'MISSING'}] {label:22s} {path}")
assert ok, "missing prims -- NOT saving"

ctx.save_stage()
print("SAVED", USD)
simulation_app.close()
