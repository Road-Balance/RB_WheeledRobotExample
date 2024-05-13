# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.physics_context.physics_context import PhysicsContext

from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup

import numpy as np


class KayaRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"

        self._wheeled_robot = world.scene.add(
            WheeledRobot(
                prim_path="/World/Kaya",
                name="my_kaya",
                wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
                create_robot=True,
                usd_path=kaya_asset_path,
                position=np.array([0, 0.0, 0.02]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        )

        self._save_count = 0

        return

    async def setup_post_load(self):
        self._world = self.get_world()
        
        kaya_setup = HolonomicRobotUsdSetup(
            robot_prim_path=self._wheeled_robot.prim_path, com_prim_path="/World/Kaya/base_link/control_offset"
        )

        (
            wheel_radius,
            wheel_positions,
            wheel_orientations,
            mecanum_angles,
            wheel_axis,
            up_axis,
        ) = kaya_setup.get_holonomic_controller_params()

        self._holonomic_controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis,
            up_axis=up_axis,
        )

        print("wheel_radius : ", wheel_radius)
        print("wheel_positions : ", wheel_positions)
        print("wheel_orientations : ", wheel_orientations)
        print("mecanum_angles : ", mecanum_angles)
        print("wheel_axis : ", wheel_axis)
        print("up_axis : ", up_axis)
        
        self._holonomic_controller.reset()

        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return

    def send_robot_actions(self, step_size):

        self._save_count += 1
        wheel_action = None

        if self._save_count >= 0 and self._save_count < 300:
            wheel_action = self._holonomic_controller.forward(command=[0.5, 0.0, 0.0])
        elif self._save_count >= 300 and self._save_count < 600:
            wheel_action = self._holonomic_controller.forward(command=[-0.5, 0.0, 0.0])
        elif self._save_count >= 600 and self._save_count < 900:
            wheel_action = self._holonomic_controller.forward(command=[0.0, 0.5, 0.0])
        elif self._save_count >= 900 and self._save_count < 1200:
            wheel_action = self._holonomic_controller.forward(command=[0.0, -0.5, 0.0])
        elif self._save_count >= 1200 and self._save_count < 1500:
            wheel_action = self._holonomic_controller.forward(command=[0.0, 0.0, 0.2])
        elif self._save_count >= 1500 and self._save_count < 1800:
            wheel_action = self._holonomic_controller.forward(command=[0.0, 0.0, -0.2])
        else:
            self._save_count = 0

        self._wheeled_robot.apply_wheel_actions(wheel_action)

        return