from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    gps_node = Node(
        package='gps_driver',
        executable='driver',
        name='gps_driver',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'frame_id': 'GPS1_Frame'},
            {'topic': '/gps'}
        ]
    )

    record_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'sqlite3',
            '/gps',
            '-o', 'bags/gps_walk'
        ],
        output='screen'
    )

    return LaunchDescription([
        gps_node,
        record_bag
    ])
