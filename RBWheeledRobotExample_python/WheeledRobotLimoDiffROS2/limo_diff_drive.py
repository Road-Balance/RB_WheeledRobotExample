# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.physics_context.physics_context import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path, get_url_root

import omni.graph.core as og

import numpy as np
import usdrt.Sdf
import carb

class LimoDiffDrive(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        carb.log_info("Check /persistent/isaac/asset_root/default setting")
        default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
        self._server_root = get_url_root(default_asset_root)

        # self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/limo_base.usd"
        self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/limo_diff_thin.usd"

        self._domain_id = 30
        self._maxLinearSpeed = 1e6
        self._wheelDistance = 0.43
        self._wheelRadius = 0.045
        self._front_jointNames = ["rear_left_wheel", "rear_right_wheel"]
        self._rear_jointNames = ["front_left_wheel", "front_right_wheel"]
        self._contorl_targetPrim = "/World/Limo/base_link"
        self._odom_targetPrim = "/World/Limo/base_footprint"

        return

    def og_setup(self):

        try:
            # OG reference : https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html
            og.Controller.edit(
                {"graph_path": "/ROS2DiffDrive", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("subscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("scaleToFromStage", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("diffController", "omni.isaac.wheeled_robots.DifferentialController"),
                        ("artControllerRear", "omni.isaac.core_nodes.IsaacArticulationController"),
                        ("artControllerFront", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("context.inputs:domain_id", self._domain_id),
                        ("diffController.inputs:maxLinearSpeed", self._maxLinearSpeed),
                        ("diffController.inputs:wheelDistance", self._wheelDistance),
                        ("diffController.inputs:wheelRadius", self._wheelRadius),
                        ("artControllerRear.inputs:jointNames", self._front_jointNames),
                        ("artControllerRear.inputs:targetPrim", [usdrt.Sdf.Path(self._contorl_targetPrim)]),
                        ("artControllerRear.inputs:usePath", False),
                        ("artControllerFront.inputs:jointNames", self._rear_jointNames),
                        ("artControllerFront.inputs:targetPrim", [usdrt.Sdf.Path(self._contorl_targetPrim)]),
                        ("artControllerFront.inputs:usePath", False),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "artControllerRear.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "artControllerFront.inputs:execIn"),
                        ("context.outputs:context", "subscribeTwist.inputs:context"),
                        ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                        ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                        ("subscribeTwist.outputs:linearVelocity", "scaleToFromStage.inputs:value"),
                        ("scaleToFromStage.outputs:result", "breakLinVel.inputs:tuple"),
                        ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                        ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                        ("diffController.outputs:velocityCommand", "artControllerRear.inputs:velocityCommand"),
                        ("diffController.outputs:velocityCommand", "artControllerFront.inputs:velocityCommand"),
                    ],
                },
            )

            # OG reference : https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_tf.html
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
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(self._odom_targetPrim)]),
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

        add_reference_to_stage(usd_path=self._robot_path, prim_path="/World/Limo")

        self._save_count = 0

        self.og_setup()
        return

    async def setup_post_load(self):
        self._world = self.get_world()
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