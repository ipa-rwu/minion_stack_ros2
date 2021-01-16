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
    bringup_dir = get_package_share_directory('minion_bringup')
    this_launch_dir = os.path.join(bringup_dir, 'launch')

    tele_pkg_dir = LaunchConfiguration(
        'tele_pkg_dir',
        default=os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch'))
        
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    joy_config = LaunchConfiguration('joy_config', default='xbox')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument(
        #     'tb3_param_dir',
        #     default_value=tb3_param_dir,
        #     description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [this_launch_dir, '/tank_hardware.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [tele_pkg_dir, '/teleop-launch.py']),
            launch_arguments={'joy_config': joy_config}.items(),
        ),

    ])