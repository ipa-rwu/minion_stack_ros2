import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"
    
    params_file = LaunchConfiguration('params_file')
    bringup_dir = get_package_share_directory('tank_driver_ros2')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'config.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='tank_driver_ros2',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=[params_file],
            output='screen',
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose')
            # ]
        )
    ])  