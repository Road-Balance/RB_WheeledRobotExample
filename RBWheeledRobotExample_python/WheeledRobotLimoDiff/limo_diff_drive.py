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
from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
from omni.isaac.core.physics_context.physics_context import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path, get_url_root

from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

import numpy as np
import carb

class LimoDiffDrive(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        carb.log_info("Check /persistent/isaac/asset_root/default setting")
        default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
        self._server_root = get_url_root(default_asset_root)

        # self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/limo_base.usd"
        self._robot_path = self._server_root + "/Projects/RBROS2/WheeledRobot/limo_diff_thin.usd"

        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        add_reference_to_stage(usd_path=self._robot_path, prim_path="/World/Limo")

        # Reference : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.wheeled_robots/docs/index.html?highlight=wheeledrobot#omni.isaac.wheeled_robots.robots.WheeledRobot
        self._wheeled_robot = world.scene.add(
            WheeledRobot(
                prim_path="/World/Limo/base_link",
                name="my_limo",
                # Caution. Those are DOF "Joints", Not "Links"
                wheel_dof_names=[
                    "front_left_wheel", 
                    "front_right_wheel",
                    "rear_left_wheel",
                    "rear_right_wheel",
                ],
                create_robot=False,
                usd_path=self._robot_path,
                position=np.array([0, 0.0, 0.02]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        )

        self._save_count = 0

        self._scene = PhysicsContext()
        self._scene.set_physics_dt(1 / 30.0)

        return

    async def setup_post_load(self):
        
        self._world = self.get_world()

        # Reference : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.wheeled_robots/docs/index.html?highlight=differentialcontroller
        self._diff_controller = DifferentialController(
            name="simple_control",
            wheel_radius=0.045,
            # Caution. This will not be the same with a real wheelbase for 4WD cases.
            # Reference : https://forums.developer.nvidia.com/t/how-to-drive-clearpath-jackal-via-ros2-messages-in-isaac-sim/275907/4
            wheel_base=0.43
        )
        self._diff_controller.reset()
        self._wheeled_robot.initialize()

        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)

        return

    def send_robot_actions(self, step_size):

        self._save_count += 1
        
        wheel_action = None

        # linear X, angular Z commands
        if self._save_count >= 0 and self._save_count < 150:
            wheel_action = self._diff_controller.forward(command=[0.3, 0.0])
        elif self._save_count >= 150 and self._save_count < 300:
            wheel_action = self._diff_controller.forward(command=[-0.3, 0.0])
        elif self._save_count >= 300 and self._save_count < 450:
            wheel_action = self._diff_controller.forward(command=[0.0, 0.3])
        elif self._save_count >= 450 and self._save_count < 600:
            wheel_action = self._diff_controller.forward(command=[0.0, -0.3])
        else:
            self._save_count = 0

        wheel_action.joint_velocities = np.hstack((wheel_action.joint_velocities, wheel_action.joint_velocities))
        self._wheeled_robot.apply_wheel_actions(wheel_action)
        return
    
    async def setup_pre_reset(self):
        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")
        self._save_count = 0
        self._world.pause()
        return
    
    async def setup_post_reset(self):
        self._diff_controller.reset()
        await self._world.play_async()
        self._world.pause()
        return
    
    def world_cleanup(self):
        self._world.pause()
        return