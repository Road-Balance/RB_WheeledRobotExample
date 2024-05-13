# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass


import numpy as np
from omni.isaac.core.utils.types import ArticulationAction

"""
This scenario takes in a robot Articulation and makes it move through its joint DOFs.
Additionally, it adds a cuboid prim to the stage that moves in a circle around the robot.

The particular framework under which this scenario operates should not be taken as a direct
recomendation to the user about how to structure their code.  In the simple example put together
in this template, this particular structure served to improve code readability and separate
the logic that runs the example from the UI design.
"""


class ExampleScenario(ScenarioTemplate):
    def __init__(self):
        self._object = None
        self._articulation = None

        self._running_scenario = False

        self._time = 0.0  # s

        self._object_radius = 0.5  # m
        self._object_height = 0.5  # m
        self._object_frequency = 0.25  # Hz

        self._joint_index = 0
        self._max_joint_speed = 4  # rad/sec
        self._lower_joint_limits = None
        self._upper_joint_limits = None

        self._joint_time = 0
        self._path_duration = 0
        self._calculate_position = lambda t, x: 0
        self._calculate_velocity = lambda t, x: 0

    def setup_scenario(self, articulation, object_prim):
        self._articulation = articulation
        self._object = object_prim

        self._initial_object_position = self._object.get_world_pose()[0]
        self._initial_object_phase = np.arctan2(self._initial_object_position[1], self._initial_object_position[0])
        self._object_radius = np.linalg.norm(self._initial_object_position[:2])

        self._running_scenario = True

        self._joint_index = 0
        self._lower_joint_limits = articulation.dof_properties["lower"]
        self._upper_joint_limits = articulation.dof_properties["upper"]

        # teleport robot to lower joint range
        epsilon = 0.001
        articulation.set_joint_positions(self._lower_joint_limits + epsilon)

        self._derive_sinusoid_params(0)

    def teardown_scenario(self):
        self._time = 0.0
        self._object = None
        self._articulation = None
        self._running_scenario = False

        self._joint_index = 0
        self._lower_joint_limits = None
        self._upper_joint_limits = None

        self._joint_time = 0
        self._path_duration = 0
        self._calculate_position = lambda t, x: 0
        self._calculate_velocity = lambda t, x: 0

    def update_scenario(self, step: float):
        if not self._running_scenario:
            return

        self._time += step

        x = self._object_radius * np.cos(self._initial_object_phase + self._time * self._object_frequency * 2 * np.pi)
        y = self._object_radius * np.sin(self._initial_object_phase + self._time * self._object_frequency * 2 * np.pi)
        z = self._initial_object_position[2]

        self._object.set_world_pose(np.array([x, y, z]))

        self._update_sinusoidal_joint_path(step)

    def _derive_sinusoid_params(self, joint_index: int):
        # Derive the parameters of the joint target sinusoids for joint {joint_index}
        start_position = self._lower_joint_limits[joint_index]

        P_max = self._upper_joint_limits[joint_index] - start_position
        V_max = self._max_joint_speed
        T = P_max * np.pi / V_max

        # T is the expected time of the joint path

        self._path_duration = T
        self._calculate_position = (
            lambda time, path_duration: start_position
            + -P_max / 2 * np.cos(time * 2 * np.pi / path_duration)
            + P_max / 2
        )
        self._calculate_velocity = lambda time, path_duration: V_max * np.sin(2 * np.pi * time / path_duration)

    def _update_sinusoidal_joint_path(self, step):
        # Update the target for the robot joints
        self._joint_time += step

        if self._joint_time > self._path_duration:
            self._joint_time = 0
            self._joint_index = (self._joint_index + 1) % self._articulation.num_dof
            self._derive_sinusoid_params(self._joint_index)

        joint_position_target = self._calculate_position(self._joint_time, self._path_duration)
        joint_velocity_target = self._calculate_velocity(self._joint_time, self._path_duration)

        action = ArticulationAction(
            np.array([joint_position_target]),
            np.array([joint_velocity_target]),
            joint_indices=np.array([self._joint_index]),
        )
        self._articulation.apply_action(action)
