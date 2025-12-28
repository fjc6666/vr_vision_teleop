import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. 准备路径
    # ---------------------------------------------------------
    franka_desc_pkg = get_package_share_directory("franka_description")
    urdf_path = os.path.join(franka_desc_pkg, "robots/fr3/fr3.urdf.xacro")
    
    my_pkg = get_package_share_directory("vr_vision_teleop")
    srdf_path = os.path.join(my_pkg, "config/fr3.srdf")
    joint_limits_path = os.path.join(my_pkg, "config/joint_limits.yaml")

    # ---------------------------------------------------------
    # 2. 配置 MoveIt
    # ---------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="franka_fr3_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={"hand": "true"} 
        )
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path="config/fr3_controllers.yaml")
        .joint_limits(file_path=joint_limits_path)
        # ！！！核心修改在这里！！！
        # 强制只加载 OMPL 规划器，彻底禁用 Pilz，这样它就不找 Pilz 的文件了
        .planning_pipelines(pipelines=["ompl"]) 
        .to_moveit_configs()
    )

    # ---------------------------------------------------------
    # 3. 启动节点
    # ---------------------------------------------------------
    planner_node = Node(
        package="vr_vision_teleop",
        executable="robot_planner_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        planner_node
    ])