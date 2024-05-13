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

class LimoAckermann(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        carb.log_info("Check /persistent/isaac/asset_root/default setting")
        default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
        self._server_root = get_url_root(default_asset_root)

        self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/limo_ackermann.usd"

        self._domain_id = 30
        self._maxWheelRotation = 1e6
        self._maxWheelVelocity = 1e6
        self._trackWidth = 0.13
        self._turningWheelRadius = 0.045
        self._wheelBase = 0.2

        self._targetPrim = "/World/Limo/base_link"
        self._robotPath = "/World/Limo/base_link"

        return

    def og_setup(self):

        try:
            og.Controller.edit(
                {"graph_path": "/ROS2Ackermann", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("subscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("scaleToFromStage", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                        ("angVelBreak", "omni.graph.nodes.BreakVector3"),
                        ("linVelBreak", "omni.graph.nodes.BreakVector3"),
                        ("wheelbase", "omni.graph.nodes.ConstantDouble"),
                        ("multiply", "omni.graph.nodes.Multiply"),
                        ("atan2", "omni.graph.nodes.ATan2"),
                        ("toRad", "omni.graph.nodes.ToRad"),
                        ("ackermannCtrl", "omni.isaac.wheeled_robots.AckermannSteering"),
                        ("wheelJointNames", "omni.graph.nodes.ConstructArray"),
                        ("wheelRotationVel", "omni.graph.nodes.ConstructArray"),
                        ("hingeJointNames", "omni.graph.nodes.ConstructArray"),
                        ("hingePosVel", "omni.graph.nodes.ConstructArray"),
                        ("articulationRotation", "omni.isaac.core_nodes.IsaacArticulationController"),
                        ("articulationPosition", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("context.inputs:domain_id", self._domain_id),
                        ("subscribeTwist.inputs:topicName", "cmd_vel"),
                        ("wheelbase.inputs:value", self._wheelBase),
                        ("ackermannCtrl.inputs:maxWheelRotation", self._maxWheelRotation),
                        ("ackermannCtrl.inputs:maxWheelVelocity", self._maxWheelVelocity),
                        ("ackermannCtrl.inputs:trackWidth", self._trackWidth),
                        ("ackermannCtrl.inputs:turningWheelRadius", self._turningWheelRadius),
                        ("ackermannCtrl.inputs:useAcceleration", False),
                        ("wheelJointNames.inputs:arraySize", 4),
                        ("wheelJointNames.inputs:arrayType", "token[]"),
                        ("wheelJointNames.inputs:input0", "rear_left_wheel"),
                        ("wheelJointNames.inputs:input1", "rear_right_wheel"),
                        ("wheelJointNames.inputs:input2", "front_left_wheel"),
                        ("wheelJointNames.inputs:input3", "front_right_wheel"),
                        ("hingeJointNames.inputs:arraySize", 2),
                        ("hingeJointNames.inputs:arrayType", "token[]"),
                        ("hingeJointNames.inputs:input0", "left_steering_hinge_wheel"),
                        ("hingeJointNames.inputs:input1", "right_steering_hinge_wheel"),
                        ("wheelRotationVel.inputs:arraySize", 4),
                        ("wheelRotationVel.inputs:arrayType", "double[]"),
                        ("hingePosVel.inputs:arraySize", 2),
                        ("hingePosVel.inputs:arrayType", "double[]"),
                        ("articulationRotation.inputs:targetPrim", [usdrt.Sdf.Path(self._targetPrim)]),
                        ("articulationRotation.inputs:robotPath", self._targetPrim),
                        ("articulationRotation.inputs:usePath", False),
                        ("articulationPosition.inputs:targetPrim", [usdrt.Sdf.Path(self._targetPrim)]),
                        ("articulationPosition.inputs:robotPath", self._targetPrim),
                        ("articulationPosition.inputs:usePath", False),
                    ],
                    og.Controller.Keys.CREATE_ATTRIBUTES: [
                        ("wheelJointNames.inputs:input1", "token"),
                        ("wheelJointNames.inputs:input2", "token"),
                        ("wheelJointNames.inputs:input3", "token"),
                        ("hingeJointNames.inputs:input1", "token"),
                        ("wheelRotationVel.inputs:input1", "double"),
                        ("wheelRotationVel.inputs:input2", "double"),
                        ("wheelRotationVel.inputs:input3", "double"),
                        ("hingePosVel.inputs:input1", "double"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("context.outputs:context", "subscribeTwist.inputs:context"),
                        ("subscribeTwist.outputs:linearVelocity", "scaleToFromStage.inputs:value"),
                        ("scaleToFromStage.outputs:result", "linVelBreak.inputs:tuple"),
                        ("subscribeTwist.outputs:angularVelocity", "angVelBreak.inputs:tuple"),
                        ("subscribeTwist.outputs:execOut", "ackermannCtrl.inputs:execIn"),
                        ("angVelBreak.outputs:z", "multiply.inputs:a"),
                        ("linVelBreak.outputs:x", "ackermannCtrl.inputs:speed"),
                        ("wheelbase.inputs:value", "multiply.inputs:b"),
                        ("wheelbase.inputs:value", "ackermannCtrl.inputs:wheelBase"),
                        ("multiply.outputs:product", "atan2.inputs:a"),
                        ("linVelBreak.outputs:x", "atan2.inputs:b"),
                        ("atan2.outputs:result", "toRad.inputs:degrees"),
                        ("toRad.outputs:radians", "ackermannCtrl.inputs:steeringAngle"),
                        ("ackermannCtrl.outputs:leftWheelAngle", "hingePosVel.inputs:input0"),
                        ("ackermannCtrl.outputs:rightWheelAngle", "hingePosVel.inputs:input1"),
                        ("ackermannCtrl.outputs:wheelRotationVelocity", "wheelRotationVel.inputs:input0"),
                        ("ackermannCtrl.outputs:wheelRotationVelocity", "wheelRotationVel.inputs:input1"),
                        ("ackermannCtrl.outputs:wheelRotationVelocity", "wheelRotationVel.inputs:input2"),
                        ("ackermannCtrl.outputs:wheelRotationVelocity", "wheelRotationVel.inputs:input3"),
                        ("ackermannCtrl.outputs:execOut", "articulationRotation.inputs:execIn"),
                        ("wheelJointNames.outputs:array", "articulationRotation.inputs:jointNames"),
                        ("wheelRotationVel.outputs:array", "articulationRotation.inputs:velocityCommand"),
                        ("ackermannCtrl.outputs:execOut", "articulationPosition.inputs:execIn"),
                        ("hingeJointNames.outputs:array", "articulationPosition.inputs:jointNames"),
                        ("hingePosVel.outputs:array", "articulationPosition.inputs:positionCommand"),
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