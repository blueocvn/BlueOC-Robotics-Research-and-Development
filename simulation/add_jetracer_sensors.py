"""Add the JetRacer AI-Kit sensor suite to arm_sim.usd, headless.

Adds, on the /jetracer_docking_scene robot:
  * an RTX RPLidar (SLAMTEC RPLIDAR_S2E, 360 deg 2D) on top of the chassis
  * a ROS2 RTX-lidar ActionGraph  (/Graph/ROS_Lidar) -> LaserScan + PointCloud2
  * a front-facing Camera on the existing front_cam_mount
  * a ROS2 camera ActionGraph  (/Graph/ROS_Camera_JetRacer) -> rgb + depth

The graph node/connection layout mirrors Isaac Sim's own og_rtx_sensors.py /
og_camera shortcuts, so it matches the scene's existing ROS_Camera / ActionGraph.

Run:  cd simulation && /home/apc/isaacsim/python.sh add_jetracer_sensors.py
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, UsdGeom

enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_sim.usd")

ROBOT = "/jetracer_docking_scene"
CHASSIS = ROBOT + "/chassis"
CAM_MOUNT = ROBOT + "/front_cam_mount"

# ---- placement (metres, relative to the chassis body frame; chassis top ~z=0.025) ----
LIDAR_XYZ = Gf.Vec3d(0.0, 0.0, 0.10)      # top-centre, ~7.5 cm above the deck

# --------------------------------------------------------------------------------------
# open the stage
# --------------------------------------------------------------------------------------
ctx = omni.usd.get_context()
ctx.open_stage(USD_PATH)
simulation_app.update()
stage = ctx.get_stage()
assert stage.GetPrimAtPath(CHASSIS).IsValid(), f"missing {CHASSIS}"
assert stage.GetPrimAtPath(CAM_MOUNT).IsValid(), f"missing {CAM_MOUNT}"


def set_local_trs(prim, translate=None, orient=None):
    """Rewrite a prim's xformOpOrder to a clean translate+orient pair (double precision)."""
    xf = UsdGeom.Xformable(prim)
    # drop any pre-existing xform ops (the create command may already have authored some)
    for op in xf.GetOrderedXformOps():
        prim.RemoveProperty(op.GetOpName())
    xf.ClearXformOpOrder()
    if translate is not None:
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if orient is not None:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(orient))


# --------------------------------------------------------------------------------------
# 1) RTX RPLidar (SLAMTEC RPLIDAR_S2E) on the chassis
# --------------------------------------------------------------------------------------
_, lidar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=CHASSIS + "/RPLidar",
    parent=None,
    config="RPLIDAR_S2E",
    translation=Gf.Vec3d(0, 0, 0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # w,i,j,k ; chassis is Z-up -> horizontal scan
)
lidar_prim = lidar.GetPrim() if hasattr(lidar, "GetPrim") else lidar
LIDAR_PATH = lidar_prim.GetPath().pathString
wrapper = stage.GetPrimAtPath(CHASSIS + "/RPLidar")
# place the wrapper on the deck; keep the OmniLidar itself at identity inside it
set_local_trs(wrapper, translate=LIDAR_XYZ, orient=Gf.Quatf(1, 0, 0, 0))
set_local_trs(lidar_prim, translate=(0, 0, 0), orient=Gf.Quatf(1, 0, 0, 0))

# cosmetic puck so the sensor is visible in the viewport (matches the primitive-built car)
puck = UsdGeom.Cylinder.Define(stage, Sdf.Path(CHASSIS + "/RPLidar/lidar_body"))
puck.CreateAxisAttr("Z")
puck.CreateRadiusAttr(0.035)
puck.CreateHeightAttr(0.04)
puck.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([(0.05, 0.05, 0.05)])
UsdGeom.Xformable(puck).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))  # sit just under the sensor origin

print(f"[lidar] created {LIDAR_PATH}  ({lidar_prim.GetTypeName()})")

# --------------------------------------------------------------------------------------
# 2) ROS2 RTX-lidar ActionGraph -> LaserScan + PointCloud2
#    (mirrors isaacsim.ros2.bridge Ros2RtxLidarGraph.make_graph)
# --------------------------------------------------------------------------------------
keys = og.Controller.Keys
LIDAR_GRAPH = "/Graph/ROS_Lidar"
og.Controller.edit(
    {"graph_path": LIDAR_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("LaserScanPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ("PointCloudPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        keys.SET_VALUES: [
            ("RenderProduct.inputs:cameraPrim", LIDAR_PATH),
            ("LaserScanPublish.inputs:topicName", "scan"),
            ("LaserScanPublish.inputs:type", "laser_scan"),
            ("LaserScanPublish.inputs:frameId", "jetracer_lidar"),
            ("PointCloudPublish.inputs:topicName", "point_cloud"),
            ("PointCloudPublish.inputs:type", "point_cloud"),
            ("PointCloudPublish.inputs:frameId", "jetracer_lidar"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "LaserScanPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "LaserScanPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "LaserScanPublish.inputs:context"),
            ("RenderProduct.outputs:execOut", "PointCloudPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "PointCloudPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "PointCloudPublish.inputs:context"),
        ],
    },
)
print(f"[lidar] built {LIDAR_GRAPH}")

# --------------------------------------------------------------------------------------
# 3) front-facing Camera on the existing front_cam_mount
# --------------------------------------------------------------------------------------
CAM_PATH = CAM_MOUNT + "/FrontCamera"
cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAM_PATH))
cam.CreateFocalLengthAttr(18.0)
cam.CreateFocusDistanceAttr(400.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100000.0))
# orient so the camera's optical axis (-Z) points along robot +X (forward), up = +Z
view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 0), Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 0, 1))
cam_xf = view.GetInverse()
cam_quat = cam_xf.ExtractRotationQuat()
set_local_trs(cam.GetPrim(), translate=(0, 0, 0), orient=Gf.Quatf(cam_quat))
# sanity: local -Z should map to ~+X in the mount frame
fwd = cam_xf.TransformDir(Gf.Vec3d(0, 0, -1))
print(f"[camera] created {CAM_PATH}; forward(-Z)->{tuple(round(v,3) for v in fwd)} (expect ~(1,0,0))")

# --------------------------------------------------------------------------------------
# 4) ROS2 camera ActionGraph -> rgb + depth  (mirrors Ros2CameraGraph.make_graph)
# --------------------------------------------------------------------------------------
CAM_GRAPH = "/Graph/ROS_Camera_JetRacer"
FRAME_ID = "jetracer_front_cam"
NS = "/jetracer_front_cam"
og.Controller.edit(
    {"graph_path": CAM_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("RenderProduct.inputs:cameraPrim", CAM_PATH),
            ("CameraInfoPublish.inputs:topicName", "camera_info"),
            ("CameraInfoPublish.inputs:frameId", FRAME_ID),
            ("CameraInfoPublish.inputs:nodeNamespace", NS),
            ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ("RGBPublish.inputs:topicName", "/rgb"),
            ("RGBPublish.inputs:type", "rgb"),
            ("RGBPublish.inputs:frameId", FRAME_ID),
            ("RGBPublish.inputs:nodeNamespace", NS),
            ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
            ("DepthPublish.inputs:topicName", "/depth"),
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:frameId", FRAME_ID),
            ("DepthPublish.inputs:nodeNamespace", NS),
            ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "RGBPublish.inputs:context"),
            ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "DepthPublish.inputs:context"),
        ],
    },
)
print(f"[camera] built {CAM_GRAPH}")

# --------------------------------------------------------------------------------------
# 5) verify + save
# --------------------------------------------------------------------------------------
simulation_app.update()
checks = {
    "lidar OmniLidar": LIDAR_PATH,
    "lidar puck": CHASSIS + "/RPLidar/lidar_body",
    "lidar graph": LIDAR_GRAPH,
    "lidar LaserScan node": LIDAR_GRAPH + "/LaserScanPublish",
    "lidar PointCloud node": LIDAR_GRAPH + "/PointCloudPublish",
    "front camera": CAM_PATH,
    "camera graph": CAM_GRAPH,
    "camera RGB node": CAM_GRAPH + "/RGBPublish",
    "camera Depth node": CAM_GRAPH + "/DepthPublish",
}
ok = True
for label, path in checks.items():
    valid = stage.GetPrimAtPath(path).IsValid()
    ok = ok and valid
    print(f"  [{'OK' if valid else 'MISSING'}] {label:24s} {path}")
assert ok, "one or more expected prims are missing -- NOT saving"

ctx.save_stage()
print(f"\nSAVED {USD_PATH}")
simulation_app.close()
