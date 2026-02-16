from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RTK device",
    )

    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="4800",
        description="Serial baud rate",
    )

    rtk_node = Node(
        package="rtk_driver",
        executable="rtk_driver",
        name="rtk_driver",
        output="screen",
        arguments=[
            "--port",
            LaunchConfiguration("port"),
            "--baud",
            LaunchConfiguration("baud"),
        ],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        rtk_node,
    ])
