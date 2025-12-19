import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    #是否启动虚拟statepublisher(仿真时为true，真实机器人时为false)
    use_fake_hardware_value = "false"
    moveit_share = get_package_share_directory("arm_moveit_config")
    # 只启动 MoveIt demo：内部包含 robot_description + move_group + fake controller + RViz
    demo_launch = os.path.join(moveit_share, "launch", "demo.launch.py")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={"use_fake_hardware": use_fake_hardware_value}.items()
    )

    return LaunchDescription(
        [
            moveit_demo,
        ]
    )