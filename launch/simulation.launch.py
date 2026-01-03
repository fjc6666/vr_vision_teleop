import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 获取包路径
    pkg_franka_ranger = get_package_share_directory('franka_ranger_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. 解析 URDF 文件
    xacro_file = os.path.join(pkg_franka_ranger, 'urdf', 'mobile_manipulator.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 3. 启动 Gazebo 服务器 (world) 和 客户端 (GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    # 4. 启动 robot_state_publisher (发布 TF 树)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. 在 Gazebo 中生成机器人 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'franka_ranger_robot', # 机器人在仿真里的名字
                   '-z', '0.1'], # 稍微抬高一点出生，防止卡地里
        output='screen'
    )

    # 6. 加载控制器 (这一步很重要，否则机械臂会软掉)
    # 注意：这里假设你还没配置具体的控制器 yaml，所以先不加载 joint_trajectory_controller
    # 我们先加载 joint_state_broadcaster 看看能不能显示
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
    ])