import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # =========================
    # arm_plan_node
    # =========================
    arm_moveit_share = Path(get_package_share_directory("arm_moveit_config"))
    moveit_config = MoveItConfigsBuilder(
        "mini1", package_name="arm_moveit_config"
    ).to_moveit_configs()

    arm_plan_config = os.path.join(
        get_package_share_directory("arm_plan_node"), "config", "config.yaml"
    )
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "mini_arm_group"}
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }


    arm_plan_node = Node(
        package="arm_plan_node",
        executable="arm_plan_node",
        name="arm_plan_node",
        output="screen",
        parameters=[
            arm_plan_config,
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # =========================
    # LaunchDescription
    # =========================
    return LaunchDescription(
        [
            arm_plan_node,
        ]
    )
