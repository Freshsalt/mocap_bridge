#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class VrpnToMavros(Node):
    def __init__(self):
        super().__init__('vrpn_to_mavros')

        self.declare_parameter('vrpn_topic', '/vrpn/UAV0/pose')
        self.declare_parameter('mavros_topic', '/mavros/mocap/pose')

        vrpn_topic = self.get_parameter('vrpn_topic').value
        mavros_topic = self.get_parameter('mavros_topic').value

        self.publisher_ = self.create_publisher(
            PoseStamped,
            mavros_topic,
            50
        )

        self.subscription_ = self.create_subscription(
            PoseStamped,
            vrpn_topic,
            self.callback,
            50
        )

        self.last_pub_time = 0.0
        self.pub_interval = 1.0 / 100.0  # 100 Hz

        self.get_logger().info(
            f'VRPN → MAVROS bridge started'
        )

    def callback(self, msg):
        now = time.time()
        if now - self.last_pub_time < self.pub_interval:
            return

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'

        # ENU → PX4 NED 适配
        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = msg.pose.position.z
        out.pose.position.z = -msg.pose.position.y

        out.pose.orientation.w = msg.pose.orientation.w
        out.pose.orientation.x = msg.pose.orientation.x
        out.pose.orientation.y = msg.pose.orientation.z
        out.pose.orientation.z = -msg.pose.orientation.y

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

