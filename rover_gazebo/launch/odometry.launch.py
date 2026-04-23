#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get config file
    config_file = os.path.join(
        get_package_share_directory("rover_gazebo"), "config", "odometry.yaml"
    )

    # Odometry node (calculated from wheels) - C++ version
    odometry_node = Node(
        package="rover_gazebo",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[config_file],
    )

    # Ground truth remapper (makes absolute odometry relative to spawn position) - C++ version
    ground_truth_remapper = Node(
        package="rover_gazebo",
        executable="ground_truth_remapper_node",
        name="ground_truth_remapper",
        output="screen",
    )

    return LaunchDescription([odometry_node, ground_truth_remapper])
