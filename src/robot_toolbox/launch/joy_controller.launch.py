import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription ,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='xbox_joy_node',
        output='screen',
    )

    joy_controller = Node(
        package='robot_toolbox',
        executable='joy_controller',
        name='xbox_joy_controller',
        output='screen',

    )



    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(joy_controller)
    return ld
