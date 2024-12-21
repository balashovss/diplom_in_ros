import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    pkgPath = FindPackageShare(package='three_legged_robot').find('three_legged_robot')
    world_path = os.path.join(pkgPath, 'worlds', 'mad_world.sdf')
    urdfModelPath = os.path.join(pkgPath, 'urdf/three_wheeled.urdf.xacro')
    
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
    # Gazebo launch
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros_gz_sim'),
    #             'launch',
    #             'gz_sim.launch.py'
    #         ])
    #     ]),
    #     launch_arguments=[
    #         ("gz_args", [
    #             LaunchConfiguration('world'),
    #             '.sdf',
    #             '-v 4',
    #             '-r'
    #         ])
    #     ]
    # )
    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
                launch_arguments=[("gz_args", [world_path])])

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('three_legged_robot'),
            'config',
            'rotation_controller.yaml',
        ]
    )
    
    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers,
                '/robot_description'],
    output="both"
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Robot spawn node
    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robot', '-r'],
        output='screen'
    )

    # RQT reconfigure node
    rqt_reconfigure_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    rotation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rotation_controller',
                   '--controller-manager',
                   '/controller_manager'],
        output='screen'
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller',
                   '--controller-manager',
                   '/controller_manager'],
        output='screen'
    )
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    #     ]),
    #     launch_arguments={
    #         'slam_params_file':PathJoinSubstitution([pkgPath, 'config', 'mapper_params_online_async.yaml'])
    #     }.items()
    # )
    bridge_params = os.path.join(pkgPath,'config','gz_bridge.yaml')
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )
    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen')
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        control_node,
        gazebo_launch,
        robot_spawn_node,
        # slam_launch,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        # Launch controller spawners after joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rotation_controller_spawner, wheel_controller_spawner],
            )
        ),

        # Launch Gazebo-ROS bridge
        bridge,

        # Launch robot state publisher
        robot_state_publisher_node,

        # Launch RQT reconfigure node
        rqt_reconfigure_node,

        # Launch joint state publisher node
        joint_state_publisher_node,

        # Launch RViz
        start_rviz_cmd
    ])