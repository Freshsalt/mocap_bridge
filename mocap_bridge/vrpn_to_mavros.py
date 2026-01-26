#!/usr/bin/env python3
"""
VRPN到MAVROS的桥梁节点（用于PX4 EKF，ENU直通）

该节点订阅VRPN发布的姿态信息（ENU），
直接转发到 /mavros/vision_pose/pose，供 PX4 EKF 使用。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class VrpnToVisionPose(Node):
    """
    VRPN到MAVROS vision_pose/pose 桥梁类（ENU直通）
    功能：
    - 订阅 VRPN 的 PoseStamped（ENU）
    - 直接发布到 /mavros/vision_pose/pose
    """

    def __init__(self):
        super().__init__('vrpn_to_vision_pose')

        # 参数声明
        self.declare_parameter('vrpn_topic', '/vrpn/UAV0/pose')
        self.declare_parameter('vision_topic', '/mavros/vision_pose/pose')

        vrpn_topic = self.get_parameter('vrpn_topic').value
        vision_topic = self.get_parameter('vision_topic').value

        # Publisher: MAVROS vision_pose 输入
        self.publisher_ = self.create_publisher(PoseStamped, vision_topic, 10)

        # Subscriber: VRPN pose
        self.subscription_ = self.create_subscription(
            PoseStamped,
            vrpn_topic,
            self.callback,
            10
        )

        self.get_logger().info(
            f'VRPN → MAVROS bridge started, publishing to {vision_topic}'
        )

    def callback(self, msg: PoseStamped):
        # 直接转发
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or 'map'

        out.pose = msg.pose

        self.publisher_.publish(out)


def main():
    rclpy.init()
    node = VrpnToVisionPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
