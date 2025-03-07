import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0', description='Serial port for GPS'
    )

    gps_node = Node(
        package='gps_driver', 
        executable='driver', 
        name='gps_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 4800,  
        }]
    )

    return LaunchDescription([
        port_arg,
        gps_node
    ])
