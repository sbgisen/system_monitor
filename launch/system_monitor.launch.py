#!/usr/bin/env python

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
