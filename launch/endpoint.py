from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ros_ip = DeclareLaunchArgument(
        "ROS_IP", default_value=TextSubstitution(text="0.0.0.0")
    )

    ros_tcp_port = DeclareLaunchArgument(
        "ROS_TCP_PORT", default_value="10000"
    )

    return LaunchDescription(
        [
            ros_ip,
            ros_tcp_port,
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[{
                    "ROS_IP": LaunchConfiguration('ROS_IP'),
                    "ROS_TCP_PORT": LaunchConfiguration('ROS_TCP_PORT')
                }],
            )
        ]
    )
