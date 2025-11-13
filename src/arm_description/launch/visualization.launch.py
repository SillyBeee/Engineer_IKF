from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _setup_nodes(context):
    package_name = LaunchConfiguration("package_name").perform(context)
    urdf_relative_path = LaunchConfiguration("urdf_relative_path").perform(context)
    rviz_config_relative_path = LaunchConfiguration("rviz_config_relative_path").perform(context)

    package_share_dir = get_package_share_directory(package_name)
    urdf_path = Path(package_share_dir) / urdf_relative_path

    if not urdf_path.exists():
        raise FileNotFoundError(f"未找到 URDF 文件: {urdf_path}")

    # RViz 配置文件路径处理（支持包名寻址）
    rviz_args = []
    if rviz_config_relative_path:
        rviz_config_path = Path(package_share_dir) / rviz_config_relative_path
        if not rviz_config_path.exists():
            raise FileNotFoundError(f"未找到 RViz 配置文件: {rviz_config_path}")
        rviz_args = ["-d", str(rviz_config_path)]

    with urdf_path.open("r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/rviz_joint_states")],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            remappings=[("joint_states", "/rviz_joint_states")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "package_name",
                default_value="arm_description",
                description="包含 URDF 和 RViz 配置的 ROS 2 包名",
            ),
            DeclareLaunchArgument(
                "urdf_relative_path",
                default_value="urdf/miniarm.urdf",
                description="URDF 在 share 目录下的相对路径",
            ),
            DeclareLaunchArgument(
                "rviz_config_relative_path",
                default_value="rviz/vis.rviz",
                description="RViz 配置文件在 share 目录下的相对路径（留空则使用默认界面）",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时间",
            ),
            OpaqueFunction(function=_setup_nodes),
        ]
    )