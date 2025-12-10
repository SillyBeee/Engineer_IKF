from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mini1", package_name="arm_moveit_config").to_moveit_configs()

    # ------------------------------------------------------------
    # 以下代码逻辑源自 generate_demo_launch，但我们将其展开
    # 以便添加 use_fake_hardware 的控制逻辑
    # ------------------------------------------------------------

    launch_package_path = moveit_config.package_path
    ld = LaunchDescription()

    # 1. 新增参数：控制是否使用虚拟硬件
    # true: 仿真模式，启动 ros2_control 和 joint_state_broadcaster
    # false: 真实模式，不启动它们，避免与你的 ArmPlanNode 冲突
    ld.add_action(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="True for simulation (fake hardware), False for real robot",
        )
    )

    # 其他标准参数
    ld.add_action(DeclareLaunchArgument("db", default_value="False", description="Start database"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="False", description="Debug mode"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="True"))

    # 2. 虚拟关节 TF (如果有)
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 3. Robot State Publisher (发布 TF)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )

    # 4. Move Group (核心规划)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py")),
        )
    )

    # 5. RViz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 6. Database (可选)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # ------------------------------------------------------------
    # 关键修改区域：加上 condition=IfCondition(LaunchConfiguration("use_fake_hardware"))
    # ------------------------------------------------------------

    # 7. ros2_control_node (Fake System)
    # 只有在仿真模式下才启动
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
            condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
        )
    )

    # 8. 加载控制器 (包括 joint_state_broadcaster)
    # 只有在仿真模式下才启动。
    # spawn_controllers.launch.py 内部会启动 joint_state_broadcaster。
    # 加上这个条件后，真实模式下就不会有 joint_state_broadcaster 运行了。
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
        )
    )

    return ld
