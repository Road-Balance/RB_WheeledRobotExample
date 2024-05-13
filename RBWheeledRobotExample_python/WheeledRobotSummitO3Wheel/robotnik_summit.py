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

import numpy as np
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

        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        add_reference_to_stage(usd_path=self._robot_path, prim_path="/World/Summit")

        self._wheeled_robot = WheeledRobot(
            prim_path="/World/Summit/summit_xl_base_link",
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

        return

    async def setup_post_load(self):
        self._world = self.get_world()

        self._summit_controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=self._wheel_radius,
            wheel_positions=self._wheel_positions,
            wheel_orientations=self._wheel_orientations,
            mecanum_angles=self._mecanum_angles,
            wheel_axis=self._wheel_axis,
            up_axis=self._up_axis,
        )
        self._summit_controller.reset()
        self._wheeled_robot.initialize()
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return

    def send_robot_actions(self, step_size):

        self._save_count += 1
        wheel_action = None

        if self._save_count >= 0 and self._save_count < 150:
            wheel_action = self._summit_controller.forward(command=[0.5, 0.0, 0.0])
        elif self._save_count >= 150 and self._save_count < 300:
            wheel_action = self._summit_controller.forward(command=[-0.5, 0.0, 0.0])
        elif self._save_count >= 300 and self._save_count < 450:
            wheel_action = self._summit_controller.forward(command=[0.0, 0.5, 0.0])
        elif self._save_count >= 450 and self._save_count < 600:
            wheel_action = self._summit_controller.forward(command=[0.0, -0.5, 0.0])
        elif self._save_count >= 600 and self._save_count < 750:
            wheel_action = self._summit_controller.forward(command=[0.0, 0.0, 0.3])
        elif self._save_count >= 750 and self._save_count < 900:
            wheel_action = self._summit_controller.forward(command=[0.0, 0.0, -0.3])
        else:
            self._save_count = 0

        self._wheeled_robot.apply_wheel_actions(wheel_action)
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