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
    # Launch 参数
    # =========================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="是否启动 RViz 可视化（true/false）",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    # =========================
    # robot_description: 通过 xacro 生成 URDF
    # =========================
    arm_description_share = get_package_share_directory("arm_moveit_config")
    # 注意：这里的文件名要和你包里的实际文件一致
    urdf_file = Path(arm_description_share) / "config" / "mini1.urdf.xacro"

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", str(urdf_file)]),
            }
        ],
    )

    # =========================
    # MoveIt 后端：move_group
    # 通过 arm_moveit_config 自带的 move_group.launch.py 启动
    # =========================
    moveit_share = get_package_share_directory("arm_moveit_config")
    move_group_launch_file = os.path.join(
        moveit_share, "launch", "move_group.launch.py"
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file)
    )

    # =========================
    # 可选：RViz 可视化（通过 moveit_rviz.launch.py）
    # =========================
    rviz_launch_file = os.path.join(
        moveit_share, "launch", "moveit_rviz.launch.py"
    )

    rviz_conditional = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch_file)
            )
        ],
        condition=IfCondition(use_rviz),
    )

    # =========================
    # 你的 arm_plan_node
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
            use_rviz_arg,
            robot_state_publisher_node,
            move_group,
            rviz_conditional,
            arm_plan_node,
        ]
    )