# VR Vision Teleop for Franka FR3

Control a Franka FR3 robot using VR controller inputs via ROS 2 Humble & MoveIt 2.

## Features
- Real-time VR target tracking (Python Bridge)
- MoveIt 2 Motion Planning (C++ Planner)
- Visual Markers in RViz

## How to Run

1. **Launch Simulation:**
   ```bash
   ros2 launch franka_fr3_moveit_config demo.launch.py
2.**Start VR Bridge:**
   ros2 run vr_vision_teleop vr_bridge.py
3.**Start Planner:**
ros2 launch vr_vision_teleop start_planner.launch.py
