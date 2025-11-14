import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 是否启动 RViz（用于单独启动 moveit_rviz.launch.py；demo 里自带 RViz，可视情况使用）
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="是否启动额外的 MoveIt RViz（true/false）",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_share = get_package_share_directory("arm_moveit_config")

    # 1) MoveIt demo：内部包含 robot_description + move_group + fake controller (+ RViz)
    demo_launch = os.path.join(moveit_share, "launch", "demo.launch.py")
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch)
    )

    # 2) 可选：额外启动一个 moveit_rviz.launch.py（如果你不想依赖 demo 里面的 RViz，可以先把 demo 里的关掉）
    rviz_launch = os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            moveit_demo,   # 内含 fake controller + move_group + robot_description (+ RViz)
            moveit_rviz,   # 可选额外 RViz
        ]
    )