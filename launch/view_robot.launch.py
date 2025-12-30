import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 1. 新增：引入 ParameterValue 用于强制类型转换
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare("franka_ranger_sim")
    
    # 生成 xacro 命令
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "mobile_manipulator.urdf.xacro"]),
        ]
    )

    # 2. 修改：用 ParameterValue 把内容包起来，告诉系统这是 str (字符串)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        # 启动关节控制滑块 GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[robot_description],  # <--- 把模型数据传给它！
        ),
        # 发布机器人状态 (TF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        # 启动 RViz
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])