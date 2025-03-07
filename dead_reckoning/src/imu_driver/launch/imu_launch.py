import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0', description='Serial port for IMU'
    )

    imu_node = Node(
        package='imu_driver', 
        executable='driver', 
        name='imu_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 115200,  
        }]
    )

    return LaunchDescription([
        port_arg,
        imu_node
    ])