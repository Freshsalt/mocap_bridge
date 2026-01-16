#!/usr/bin/env python3
"""
VRPN到MAVROS的桥梁节点

该节点订阅VRPN发布的姿态信息，并将其转换为MAVROS可接受的格式。
主要功能是将ENU坐标系转换为PX4使用的NED坐标系。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class VrpnToMavros(Node):
    """
    VRPN到MAVROS的桥梁类
    
    该类创建一个ROS 2节点，用于接收VRPN的姿态数据并将其转换为MAVROS格式，
    同时处理坐标系转换（从ENU到NED）和发布频率控制。
    """
    
    def __init__(self):
        """初始化VRPN到MAVROS的桥梁节点"""
        super().__init__('vrpn_to_mavros')

        # 声明参数：VRPN话题名称、MAVROS话题名称和位置变化阈值
        self.declare_parameter('vrpn_topic', '/vrpn/UAV0/pose')
        self.declare_parameter('mavros_topic', '/mavros/mocap/pose')
        self.declare_parameter('position_threshold', 1.0)  # 位置变化阈值，单位米

        # 获取参数值
        vrpn_topic = self.get_parameter('vrpn_topic').value
        mavros_topic = self.get_parameter('mavros_topic').value
        self.position_threshold = self.get_parameter('position_threshold').value

        # 创建发布者：向MAVROS发布姿态信息
        self.publisher_ = self.create_publisher(
            PoseStamped,
            mavros_topic,
            50  # 队列大小
        )

        # 创建订阅者：订阅VRPN发布的姿态信息
        self.subscription_ = self.create_subscription(
            PoseStamped,
            vrpn_topic,
            self.callback,
            50  # 队列大小
        )

        # 初始化时间变量，用于控制发布频率
        self.last_pub_time = 0.0
        self.pub_interval = 1.0 / 50.0  # 发布间隔，50Hz
        
        # 存储上一次有效的位置数据，用于滤波
        self.last_valid_pose = None

        # 输出启动信息
        self.get_logger().info(
            f'VRPN → MAVROS bridge started with position threshold: {self.position_threshold}m'
        )

    def callback(self, msg):
        """
        VRPN姿态消息回调函数
        
        接收来自VRPN的PoseStamped消息，进行坐标系转换后发布到MAVROS。
        包含滤波逻辑，过滤掉位置变化过大的异常数据。
        
        参数:
            msg: 来自VRPN的PoseStamped消息
        """
        # 获取当前时间
        now = time.time()
        
        # 控制发布频率，确保不超过设定的最大频率
        if now - self.last_pub_time < self.pub_interval:
            return

        # 检查位置变化是否过大，如果是则使用上一次的有效数据
        filtered_msg = msg
        if self.last_valid_pose is not None:
            # 计算当前位置与上一次有效位置的距离
            dx = msg.pose.position.x - self.last_valid_pose.pose.position.x
            dy = msg.pose.position.y - self.last_valid_pose.pose.position.y
            dz = msg.pose.position.z - self.last_valid_pose.pose.position.z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            
            if distance > self.position_threshold:
                # 如果距离变化超过阈值，则使用上一次的有效数据
                self.get_logger().warn(f'Position jump detected: {distance:.3f}m > {self.position_threshold}m, using previous pose')
                filtered_msg = self.last_valid_pose
            else:
                # 否则更新有效位置
                self.last_valid_pose = msg
        else:
            # 第一次接收到数据，直接保存为有效位置
            self.last_valid_pose = msg

        # 创建输出的PoseStamped消息
        out = PoseStamped()
        # 设置时间戳为当前时间
        out.header.stamp = self.get_clock().now().to_msg()
        # 设置参考坐标系
        out.header.frame_id = 'map'

        # 进行坐标系转换：从ENU(东-北-上)转换为NED(北-东-下)
        # ENU → PX4 NED 适配
        out.pose.position.x = filtered_msg.pose.position.x  # E → N
        out.pose.position.y = filtered_msg.pose.position.z  # U → D (注意这里是负号，在下面体现)
        out.pose.position.z = -filtered_msg.pose.position.y  # N → E (注意这里是负号)

        # 转换方向四元数：从ENU到NED
        out.pose.orientation.w = filtered_msg.pose.orientation.w  # 标量部分不变
        out.pose.orientation.x = filtered_msg.pose.orientation.x  # x轴分量对应关系
        out.pose.orientation.y = filtered_msg.pose.orientation.z  # y轴分量对应关系
        out.pose.orientation.z = -filtered_msg.pose.orientation.y  # z轴分量对应关系(带符号变化)

        # 发布转换后的姿态信息
        self.publisher_.publish(out)
        # 更新上次发布时间
        self.last_pub_time = now


def main():
    """
    主函数
    
    初始化ROS 2系统，创建节点实例，开始处理消息循环。
    """
    # 初始化ROS 2
    rclpy.init()
    # 创建桥梁节点
    node = VrpnToMavros()
    # 开始消息循环
    rclpy.spin(node)
    # 销毁节点
    node.destroy_node()
    # 关闭ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()

