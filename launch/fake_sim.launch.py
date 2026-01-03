import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 1. 新增：引入 ParameterValue
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    pkg_share = FindPackageShare("franka_ranger_sim")
    
    # URDF 文件路径
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "mobile_manipulator_pure.urdf"])

    # 2. 关键修改：用 ParameterValue 包裹 Command，并指定 value_type=str
    robot_description_content = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    
    # 构造参数字典
    robot_description = {"robot_description": robot_description_content}

    # 控制器配置文件路径
    controller_config = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    # =========================================================
    # 节点定义
    # =========================================================

    # A. 机器人状态发布者
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # B. Controller Manager (核心：加载 Mock 硬件)
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # C. 加载控制器：Joint State Broadcaster
    spawner_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # D. 加载控制器：Arm Controller
    spawner_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller"],
        output="screen",
    )

    # E. RViz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        # 如果你还没有保存过 rviz 配置，建议先注释掉下面这行，等 RViz 打开后手动添加 RobotModel 再保存
        # arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "view_robot.rviz"])],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_rviz,
        
        # 顺序控制：Controller Manager 启动后 -> Joint State -> Arm Controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawner_joint_state,
                on_exit=[spawner_arm_controller],
            )
        ),
        spawner_joint_state,
    ])