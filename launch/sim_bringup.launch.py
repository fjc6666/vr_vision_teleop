import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    pkg_share = FindPackageShare("franka_ranger_sim")
    gazebo_ros_pkg = FindPackageShare("gazebo_ros")
    
    # 1. 准备路径
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "mobile_manipulator_pure.urdf"])
    # 强制获取 yaml 路径，不依赖 URDF 里的写法
    controller_config = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    # 2. 读取 URDF
    robot_description = {
        "robot_description": ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    }

    # 3. 启动 Gazebo
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_pkg, "launch", "gzserver.launch.py"])
        )
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_pkg, "launch", "gzclient.launch.py"])
        )
    )

    # 4. 发布 TF (关键：设置 use_sim_time = True)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 5. 生成机器人 (Spawn)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "mobile_manipulator",
            "-z", "0.1",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 6. 加载控制器 (关键：设置 use_sim_time = True)
    # 只要这步成功，RViz 就不会报错了
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 7. 启动 RViz (顺便帮你配置好)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        # 如果你没保存过 rviz 配置，可以把下面这行注释掉
        # arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "view_robot.rviz"])],
    )

    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
        node_rviz,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])