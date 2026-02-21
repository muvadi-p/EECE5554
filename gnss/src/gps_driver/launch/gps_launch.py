from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='GPS serial port (e.g., /dev/ttyUSB2 or /dev/pts/6)'
    )

    port = LaunchConfiguration('port')

    return LaunchDescription([
        port_arg,
        Node(
            package='gps_driver',
            executable='driver',
            name='gps_driver',
            output='screen',
            arguments=['--port', port],
        )
    ])
