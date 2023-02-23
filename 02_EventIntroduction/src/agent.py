#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Agent for simulator interface to Self-Driving Car Controller

Copyright (c) 2023 Toyota Tsusho Denso Electronics (Thailand) Co. Ltd and Toyota Tsusho Nexty Electronics (Thailand) Co. Ltd
All rights reserved.

Developed by:
    Jeerapat Jitnuant
    Software Engineer, Software Development Group
    Toyota Tsusho Denso Electronics (Thailand) Co. Ltd
    jeerapat.j.itnuant.a7m@ap.denso.com

This work is licensed under the terms of the Toyota Tsusho Denso Electronics (Thailand) Co. Ltd and Toyota Tsusho Nexty Electronics (Thailand) Co. Ltd
"""

from __future__ import print_function

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
import numpy as np


def get_entry_point():
    return 'CarlaAgent'


class CarlaAgent(AutonomousAgent):

    """
    Dummy autonomous agent to control the ego vehicle
    """

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """

    def run_step(self, input_data, timestamp):
        # self._global_plan is the mission route

        # Transform GNSS position into meter unit, (x, y, z)
        position = np.array([input_data['GPS'][1][0]*139000, input_data['GPS'][1][1]*111000, input_data['GPS'][1][2]])

        # Calculate heading angle from radian in degree
        compass = (180 * input_data['IMU'][1][6]) / np.pi   # heading angle in degree

        wheel_speed = input_data['SPEED'][1]['speed']       # Linear velocity in meter per second.
        imu_f = np.array(input_data['IMU'][1][0:3])         # acceleration, (fx, fy, fz) in m^2/sec
        imu_w = np.array(input_data['IMU'][1][3:6])         # gyroscope, (wx, wy, wz) in rad/sec
        camera = input_data['CAMERA'][1][:, :, :3]          # Front camera data
        semantic_segmentation = input_data['SEMANTIC_SEGMENTATION_CAMERA'][1][:, :, 2]  # Front camera data after Semantic segmentation transform
        radar = input_data['RADAR'][1]                      # (depth, elevation angle, azimuth angle, velocity)
        radar[:, [1, 2]] = radar[:, [2, 1]]                 # Relocate the RADAR data position to (depth, azimuth angle, elevation angle, velocity)

        sensor_data = {
            'timestamp': timestamp,
            'position': position,
            'wheel_speed': wheel_speed,
            'compass': compass,
            'imu_f': imu_f,
            'imu_w': imu_w,
            'camera': camera,
            'semantic_segmentation': semantic_segmentation,
            'radar': radar,
        }

        control = carla.VehicleControl()

        control.steer = 0               # A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.
        control.throttle = 0            # A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.
        control.brake = 0               # A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.
        control.hand_brake = False      # Determines whether hand brake will be used. Default is False.
        control.reverse = False         # Determines whether the vehicle will move backwards. Default is False.

        return control
