import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # port_arg = DeclareLaunchArgument(
    #     'port', default_value='/dev/ttyUSB0', description='Serial port for GPS'
    # )

    gps_node = Node(
        package='gps_driver', 
        executable='driver', 
        name='gps_node',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baudrate': 4800,  
        }]
    )

    imu_node = Node(
        package='imu_driver', 
        executable='driver', 
        name='imu_node',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,  
        }]
    )

    return LaunchDescription([
        gps_node,
        imu_node
    ])
