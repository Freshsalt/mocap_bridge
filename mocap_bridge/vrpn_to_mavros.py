#!/usr/bin/env python3
"""
VRPN到MAVROS的桥梁节点（不做坐标系转换）

该节点订阅VRPN发布的姿态信息（ENU），
直接转发到 /mavros/mocap/pose，
由 MAVROS 负责 ENU → NED 的转换与 MAVLink 发送。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class VrpnToMavros(Node):
    """
    VRPN到MAVROS的桥梁类（ENU直通）

    功能：
    - 订阅 VRPN 的 PoseStamped（ENU）
    - 做位置跳变滤波
    - 控制发布频率
    - 原样发布到 /mavros/mocap/pose
    """

    def __init__(self):
        super().__init__('vrpn_to_mavros')

        # 参数声明
        self.declare_parameter('vrpn_topic', '/vrpn/UAV0/pose')
        self.declare_parameter('mavros_topic', '/mavros/mocap/pose')
        self.declare_parameter('position_threshold', 0.5)

        vrpn_topic = self.get_parameter('vrpn_topic').value
        mavros_topic = self.get_parameter('mavros_topic').value
        self.position_threshold = self.get_parameter('position_threshold').value

        # Publisher: MAVROS mocap 输入
        self.publisher_ = self.create_publisher(
            PoseStamped,
            mavros_topic,
            50
        )

        # Subscriber: VRPN pose
        self.subscription_ = self.create_subscription(
            PoseStamped,
            vrpn_topic,
            self.callback,
            50
        )

        # 频率控制
        self.last_pub_time = 0.0
        self.pub_interval = 1.0 / 50.0  # 50 Hz

        # 上一次有效位姿
        self.last_valid_pose = None

        self.get_logger().info(
            f'VRPN → MAVROS bridge started (ENU passthrough), '
            f'position threshold: {self.position_threshold} m'
        )

    def callback(self, msg: PoseStamped):
        now = time.time()

        # 限频
        if now - self.last_pub_time < self.pub_interval:
            return

        # 跳变滤波
        filtered_msg = msg
        if self.last_valid_pose is not None:
            dx = msg.pose.position.x - self.last_valid_pose.pose.position.x
            dy = msg.pose.position.y - self.last_valid_pose.pose.position.y
            dz = msg.pose.position.z - self.last_valid_pose.pose.position.z
            distance = (dx * dx + dy * dy + dz * dz) ** 0.5

            if distance > self.position_threshold:
                self.get_logger().warn(
                    f'Position jump detected: {distance:.3f} m > '
                    f'{self.position_threshold} m, using previous pose'
                )
                filtered_msg = self.last_valid_pose
            else:
                self.last_valid_pose = msg
        else:
            self.last_valid_pose = msg

        # 直接转发（ENU → ENU）
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = filtered_msg.header.frame_id or 'map'

        out.pose.position.x = filtered_msg.pose.position.x
        out.pose.position.y = filtered_msg.pose.position.y
        out.pose.position.z = filtered_msg.pose.position.z

        out.pose.orientation.x = filtered_msg.pose.orientation.x
        out.pose.orientation.y = filtered_msg.pose.orientation.y
        out.pose.orientation.z = filtered_msg.pose.orientation.z
        out.pose.orientation.w = filtered_msg.pose.orientation.w

        self.publisher_.publish(out)
        self.last_pub_time = now


def main():
    rclpy.init()
    node = VrpnToMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
