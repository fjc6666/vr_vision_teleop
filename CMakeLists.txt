cmake_minimum_required(VERSION 3.8)
project(vr_vision_teleop)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs REQUIRED) 

# 2. 编译 C++ 节点 (接收端)
add_executable(robot_planner_node src/robot_planner.cpp)
ament_target_dependencies(robot_planner_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  visualization_msgs
)

# 3. 安装 C++ 节点
install(TARGETS
  robot_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 安装 Python 脚本 (发送端)
install(PROGRAMS
  scripts/vr_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
# 安装 Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 【新增】安装 Config 文件夹 (里面有 fr3.srdf)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()