from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vrpn_topic_arg = DeclareLaunchArgument(
        'vrpn_topic',
        default_value='/vrpn/UAV0/pose',
        description='Input VRPN pose topic'
    )

    mavros_topic_arg = DeclareLaunchArgument(
        'mavros_topic',
        default_value='/mavros/mocap/pose',
        description='Output MAVROS mocap topic'
    )

    vrpn_to_mavros_node = Node(
        package='mocap_bridge',
        executable='vrpn_to_mavros',
        name='vrpn_to_mavros',
        output='screen',
        parameters=[{
            'vrpn_topic': LaunchConfiguration('vrpn_topic'),
            'mavros_topic': LaunchConfiguration('mavros_topic'),
        }]
    )

    return LaunchDescription([
        vrpn_topic_arg,
        mavros_topic_arg,
        vrpn_to_mavros_node
    ])

