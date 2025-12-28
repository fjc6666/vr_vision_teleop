#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VRBridgePublisher(Node):
    def __init__(self):
        super().__init__('vr_bridge_node')

        # 创建发布者：话题名为 '/vr_target_pose'
        # 队列长度为 10，防止数据堆积
        self.publisher_ = self.create_publisher(PoseStamped, '/vr_target_pose', 10)

        # 设置发送频率（例如 60Hz，与VR刷新率匹配）
        timer_period = 1.0 / 60.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('VR Bridge Interface Started: Publishing to /vr_target_pose')

    def timer_callback(self):
        msg = PoseStamped()

        # 1. 填充时间戳和坐标系
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"  # VR数据的参考坐标系

        # 2. 填充位姿数据 (这里用模拟数据，实际项目中应替换为 VR SDK 的读数)
        # 例如：x, y, z 从 VR 手柄获取
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.4

        # 四元数 (Orientation)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        # 3. 发布消息
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VRBridgePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()