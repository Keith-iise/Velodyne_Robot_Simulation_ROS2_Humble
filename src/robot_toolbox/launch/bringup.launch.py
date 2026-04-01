import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription ,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    gui = LaunchConfiguration('gui')
    rate = LaunchConfiguration('rate')

    #生命启动参数
    declare_gui = DeclareLaunchArgument('gui' , default_value='false' , description='')
    declare_rate = DeclareLaunchArgument('rate' , default_value='200.0' , description='')
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
        'gui': gui,
        'rate': rate,
    }.items()

    )

    ld = LaunchDescription()
    ld.add_action(declare_gui)
    ld.add_action(declare_rate)
    ld.add_action(joy_controller_launch)
    ld.add_action(vehicle_simulator_launch)
    return ld
