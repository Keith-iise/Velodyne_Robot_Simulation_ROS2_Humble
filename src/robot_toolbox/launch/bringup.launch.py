import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription ,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    

    joy_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_toolbox'), 'launch', 'joy_controller.launch.py')
        ),  
    )

    vehicle_simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle_simulator.launch.py')
        ),  
        launch_arguments={
        'gui': 'True',
    }.items()

    )

    ld = LaunchDescription()
    ld.add_action(joy_controller_launch)
    ld.add_action(vehicle_simulator_launch)
    return ld
