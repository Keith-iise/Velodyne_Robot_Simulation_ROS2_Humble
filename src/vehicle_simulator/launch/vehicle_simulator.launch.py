import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription ,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

#动态声明 Gazebo 的 world 参数（Gazebo 启动时需要指定 world 参数来加载场景文件）
def declare_world_action(context,world_name):
    world_name_str = str(world_name.perform(context))
    declare_world = DeclareLaunchArgument('world',default_value=[os.path.join(get_package_share_directory('vehicle_simulator'),'world',world_name_str+'.world')],description='')
    return [declare_world]

def generate_launch_description():
    #定义参数引用
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')

    #生命启动参数
    declare_world_name = DeclareLaunchArgument('world_name' , default_value='indoor' , description='')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time' , default_value='true' , description='')

    # gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    #     launch_arguments={
    #     'on_exit_shutdown': 'True',
    #     'verbose': 'true',
    #     'extra_gzserver_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
    # }.items()
    )

    # lidar robot_state_publisher
    lidar_xacro = os.path.join(get_package_share_directory('vehicle_simulator') , 'urdf' , 'lidar.urdf.xacro')
    lidar_description = Command(['xacro' , ' ' , lidar_xacro])
    start_lidar_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': lidar_description,
        }]
    )

    #Spawn lidar
    spawn_lidar = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lidar',
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    #Spawn robot
    robot_xacro = os.path.join(get_package_share_directory('vehicle_simulator') , 'urdf' , 'robot.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-file', robot_xacro
        ],
        output='screen',

    )

    # --- 新增：静态 TF 发布节点 ---
    # 参数顺序：x y z qx qy qz qw 或 x y z yaw pitch roll
    # 这里使用的是：x y z roll pitch yaw
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0.75', '0', '0', '0', 'base_link', 'lidar'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    #Spawn camera
    camera_xacro = os.path.join(get_package_share_directory('vehicle_simulator') , 'urdf' , 'camera.urdf.xacro')
    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'camera',
            '-file', camera_xacro
        ],
        output='screen',
    )

    # vehicleSimulator
    start_vehicle_simulator = Node(
        package='vehicle_simulator',
        executable='vehicleSimulator',
        name='vehicleSimulator',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen',

    )

    # 延迟启动
    delayed_start_vehicle_simulator = TimerAction(
        period=8.0,
        actions=[start_vehicle_simulator]
    )

    delayed_spawn_entities = TimerAction(
        period=5.0,
        actions=[
            spawn_robot,
            spawn_lidar,
            spawn_camera
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_name)
    ld.add_action(OpaqueFunction(function=declare_world_action,args=[world_name]))

    ld.add_action(start_gazebo)
    ld.add_action(start_lidar_state_publisher)
    ld.add_action(static_tf_pub)
    ld.add_action(delayed_spawn_entities)
    # ld.add_action(spawn_lidar)
    # ld.add_action(spawn_robot)
    # ld.add_action(spawn_camera)
    ld.add_action(delayed_start_vehicle_simulator)
    return ld
