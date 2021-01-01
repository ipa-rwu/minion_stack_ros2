import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('rplidar_ros'), 'launch'))

    tank_driver_pkg_dir = LaunchConfiguration(
        'tank_driver_pkg_dir',
        default=os.path.join(get_package_share_directory('tank_driver_ros2'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with Lidar'),

        # DeclareLaunchArgument(
        #     'tb3_param_dir',
        #     default_value=tb3_param_dir,
        #     description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [tank_driver_pkg_dir, '/tank_driver_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [tank_driver_pkg_dir, '/imu_publsiher_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/rplidar.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf2_base_link_laser',
            arguments=["0.1", "0", "0.3", "0", "-1", "0", "0", "base_link", "laser"],
            output='screen',
            ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf2_base_link_imu',
            arguments=["0", "0", "0", "0", "1", "0", "0", "base_link", "imu_link"],
            output='screen',
            ),
    ])