import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():


    # =========================
    # arm_plan_node
    # =========================
    arm_plan_config = os.path.join(
        get_package_share_directory("arm_plan_node"), "config", "config.yaml"
    )

    arm_plan_node = Node(
        package="arm_plan_node",
        executable="arm_plan_node",
        name="arm_plan_node",
        output="screen",
        parameters=[arm_plan_config],
    )

    # =========================
    # LaunchDescription
    # =========================
    return LaunchDescription(
        [

            arm_plan_node,
        ]
    )