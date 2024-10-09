#!/usr/bin/env python

# Copyright (c) 2024 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParametersFromFile
from launch_ros.actions import SetParametersFromFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []

    args.append(
        DeclareLaunchArgument('machine_name',
                              default_value=os.environ.get('HOSTNAME', 'localhost')))
    pkg_path = FindPackageShare('system_monitor')
    args.append(
        DeclareLaunchArgument('config_file',
                              default_value=[pkg_path, '/config/system_monitor.yaml']))
    args.append(DeclareLaunchArgument('output', default_value='log'))

    node = GroupAction(actions=[
        PushRosNamespace(['system_monitor/',
                          LaunchConfiguration('machine_name')]),
        SetParametersFromFile(LaunchConfiguration('config_file')),
        Node(
            package='system_monitor',
            executable='cpu_monitor',
            name='cpu_monitor',
            output=LaunchConfiguration('output'),
        ),
        Node(
            package='system_monitor',
            executable='hdd_monitor',
            name='hdd_monitor',
            output=LaunchConfiguration('output'),
        ),
        Node(
            package='system_monitor',
            executable='mem_monitor',
            name='mem_monitor',
            output=LaunchConfiguration('output'),
        ),
        Node(
            package='system_monitor',
            executable='ntp_monitor',
            name='ntp_monitor',
            output=LaunchConfiguration('output'),
        ),
        Node(
            package='system_monitor',
            executable='net_monitor',
            name='net_monitor',
            output=LaunchConfiguration('output'),
        ),
    ])
    return LaunchDescription(args + [node])
