# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.physics_context.physics_context import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path, get_url_root

from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
import omni.graph.core as og

import numpy as np
import usdrt.Sdf
import carb

class RobotnikSummit(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        carb.log_info("Check /persistent/isaac/asset_root/default setting")
        default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
        self._server_root = get_url_root(default_asset_root)

        # wheel models referenced from : https://git.openlogisticsfoundation.org/silicon-economy/simulation-model/o3dynsimmodel
        self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/Collected_summit_xl_omni_four/summit_xl_omni_four.usd"

        self._wheel_radius = np.array([ 0.127, 0.127, 0.127, 0.127 ])
        self._wheel_positions = np.array([
            [0.229, 0.235, 0.11],
            [0.229, -0.235, 0.11],
            [-0.229, 0.235, 0.11],
            [-0.229, -0.235, 0.11],
        ])

        self._wheel_orientations = np.array([
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, -0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, -0.7071068],
        ])
        self._mecanum_angles = np.array([
            -135.0,
            -45.0,
            -45.0,
            -135.0,
        ])
        self._wheel_axis = np.array([1, 0, 0])
        self._up_axis = np.array([0, 0, 1])

        self._targetPrim = "/World/Summit/summit_xl_base_link"
        self._domain_id = 30

        return

    def og_setup(self):

        try:
            og.Controller.edit(
                {"graph_path": "/ROS2HolonomicTwist", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("subscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("scaleToFromStage", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("angvelGain", "omni.graph.nodes.ConstantDouble"),
                        ("angvelMult", "omni.graph.nodes.Multiply"),
                        ("linXGain", "omni.graph.nodes.ConstantDouble"),
                        ("linXMult", "omni.graph.nodes.Multiply"),
                        ("linYGain", "omni.graph.nodes.ConstantDouble"),
                        ("linYMult", "omni.graph.nodes.Multiply"),
                        ("velVec3", "omni.graph.nodes.MakeVector3"),
                        ("mecanumAng", "omni.graph.nodes.ConstructArray"),
                        ("holonomicCtrl", "omni.isaac.wheeled_robots.HolonomicController"),

                        ("upAxis", "omni.graph.nodes.ConstantDouble3"),
                        ("wheelAxis", "omni.graph.nodes.ConstantDouble3"),
                        ("wheelOrientation", "omni.graph.nodes.ConstructArray"),
                        ("wheelPosition", "omni.graph.nodes.ConstructArray"),   
                        ("wheelRadius", "omni.graph.nodes.ConstructArray"),   
                        ("jointNames", "omni.graph.nodes.ConstructArray"),   
                        ("articulation", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("context.inputs:domain_id", self._domain_id),
                        ("subscribeTwist.inputs:topicName", "cmd_vel"),
                        ("angvelGain.inputs:value", -0.514),
                        ("linXGain.inputs:value", 2.325),
                        ("linYGain.inputs:value", 3.0),

                        ("mecanumAng.inputs:arraySize", 4),
                        ("mecanumAng.inputs:arrayType", "double[]"),
                        ("mecanumAng.inputs:input0", self._mecanum_angles[0]),
                        ("mecanumAng.inputs:input1", self._mecanum_angles[1]),
                        ("mecanumAng.inputs:input2", self._mecanum_angles[2]),
                        ("mecanumAng.inputs:input3", self._mecanum_angles[3]),
                        ("holonomicCtrl.inputs:angularGain", 1.0),
                        ("holonomicCtrl.inputs:linearGain", 1.0),
                        ("holonomicCtrl.inputs:maxWheelSpeed", 1200.0),

                        ("upAxis.inputs:value", self._up_axis),
                        ("wheelAxis.inputs:value", self._wheel_axis),

                        ("wheelOrientation.inputs:arraySize", 4),
                        ("wheelOrientation.inputs:arrayType", "double[4][]"),
                        ("wheelOrientation.inputs:input0", self._wheel_orientations[0]),
                        ("wheelOrientation.inputs:input1", self._wheel_orientations[1]),
                        ("wheelOrientation.inputs:input2", self._wheel_orientations[2]),
                        ("wheelOrientation.inputs:input3", self._wheel_orientations[3]),

                        ("wheelPosition.inputs:arraySize", 4),
                        ("wheelPosition.inputs:arrayType", "double[3][]"),
                        ("wheelPosition.inputs:input0", self._wheel_positions[0]),
                        ("wheelPosition.inputs:input1", self._wheel_positions[1]),
                        ("wheelPosition.inputs:input2", self._wheel_positions[2]),
                        ("wheelPosition.inputs:input3", self._wheel_positions[3]),

                        ("wheelRadius.inputs:arraySize", 4),
                        ("wheelRadius.inputs:arrayType", "double[]"),
                        ("wheelRadius.inputs:input0", self._wheel_radius[0]),
                        ("wheelRadius.inputs:input1", self._wheel_radius[1]),
                        ("wheelRadius.inputs:input2", self._wheel_radius[2]),
                        ("wheelRadius.inputs:input3", self._wheel_radius[3]),

                        ("jointNames.inputs:arraySize", 4),
                        ("jointNames.inputs:arrayType", "token[]"),
                        ("jointNames.inputs:input0", "fl_joint"),
                        ("jointNames.inputs:input1", "fr_joint"),
                        ("jointNames.inputs:input2", "rl_joint"),
                        ("jointNames.inputs:input3", "rr_joint"),

                        ("articulation.inputs:targetPrim", [usdrt.Sdf.Path(self._targetPrim)]),
                        ("articulation.inputs:robotPath", self._targetPrim),
                        ("articulation.inputs:usePath", False),
                    ],
                    og.Controller.Keys.CREATE_ATTRIBUTES: [
                        ("mecanumAng.inputs:input1", "double"),
                        ("mecanumAng.inputs:input2", "double"),
                        ("mecanumAng.inputs:input3", "double"),
                        ("wheelOrientation.inputs:input1", "double[4]"),
                        ("wheelOrientation.inputs:input2", "double[4]"),
                        ("wheelOrientation.inputs:input3", "double[4]"),
                        ("wheelPosition.inputs:input1", "double[3]"),
                        ("wheelPosition.inputs:input2", "double[3]"),
                        ("wheelPosition.inputs:input3", "double[3]"),
                        ("wheelRadius.inputs:input1", "double"),
                        ("wheelRadius.inputs:input2", "double"),
                        ("wheelRadius.inputs:input3", "double"),
                        ("jointNames.inputs:input1", "token"),
                        ("jointNames.inputs:input2", "token"),
                        ("jointNames.inputs:input3", "token"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("context.outputs:context", "subscribeTwist.inputs:context"),
                        ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                        ("subscribeTwist.outputs:linearVelocity", "scaleToFromStage.inputs:value"),
                        ("scaleToFromStage.outputs:result", "breakLinVel.inputs:tuple"),

                        ("breakAngVel.outputs:z", "angvelMult.inputs:a"),
                        ("angvelGain.inputs:value", "angvelMult.inputs:b"),
                        ("breakLinVel.outputs:x", "linXMult.inputs:a"),
                        ("linXGain.inputs:value", "linXMult.inputs:b"),
                        ("breakLinVel.outputs:y", "linYMult.inputs:a"),
                        ("linYGain.inputs:value", "linYMult.inputs:b"),

                        ("angvelMult.outputs:product", "velVec3.inputs:z"),
                        ("linXMult.outputs:product", "velVec3.inputs:x"),
                        ("linYMult.outputs:product", "velVec3.inputs:y"),

                        ("onPlaybackTick.outputs:tick", "holonomicCtrl.inputs:execIn"),
                        ("velVec3.outputs:tuple", "holonomicCtrl.inputs:velocityCommands"),
                        ("mecanumAng.outputs:array", "holonomicCtrl.inputs:mecanumAngles"),
                        ("onPlaybackTick.outputs:tick", "holonomicCtrl.inputs:execIn"),

                        ("upAxis.inputs:value", "holonomicCtrl.inputs:upAxis"),
                        ("wheelAxis.inputs:value", "holonomicCtrl.inputs:wheelAxis"),
                        ("wheelOrientation.outputs:array", "holonomicCtrl.inputs:wheelOrientations"),
                        ("wheelPosition.outputs:array", "holonomicCtrl.inputs:wheelPositions"),
                        ("wheelRadius.outputs:array", "holonomicCtrl.inputs:wheelRadius"),

                        ("onPlaybackTick.outputs:tick", "articulation.inputs:execIn"),
                        ("holonomicCtrl.outputs:jointVelocityCommand", "articulation.inputs:velocityCommand"),
                        ("jointNames.outputs:array", "articulation.inputs:jointNames"),
                    ],
                },
            )

            og.Controller.edit(
                {"graph_path": "/ROS2Odom", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("computeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                        ("publishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                        ("publishRawTF", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("context.inputs:domain_id", self._domain_id),
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(self._targetPrim)]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                        ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                        ("readSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                        ("context.outputs:context", "publishOdom.inputs:context"),
                        ("context.outputs:context", "publishRawTF.inputs:context"),
                        ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                        ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                        ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                        ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                        ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                        ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),
                    ],
                },
            )

        except Exception as e:
            print(e)

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        add_reference_to_stage(usd_path=self._robot_path, prim_path="/World/Summit")

        self._wheeled_robot = WheeledRobot(
            prim_path=self._targetPrim,
            name="my_summit",
            wheel_dof_names=[
                "fl_joint",
                "fr_joint",
                "rl_joint",
                "rr_joint",
            ],
            create_robot=True,
            usd_path=self._robot_path,
            position=np.array([0, 0.0, 0.02]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        self._save_count = 0

        self._scene = PhysicsContext()
        self._scene.set_physics_dt(1 / 30.0)

        self.og_setup()
        return

    async def setup_post_load(self):
        self._world = self.get_world()

        self._wheeled_robot.initialize()
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return

    def send_robot_actions(self, step_size):
        self._save_count += 1
        return
    
    async def setup_pre_reset(self):
        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")
        self._world.pause()
        return
    
    async def setup_post_reset(self):
        self._summit_controller.reset()
        await self._world.play_async()
        self._world.pause()
        return
    
    def world_cleanup(self):
        self._world.pause()
        return