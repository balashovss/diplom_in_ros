import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    pkgPath = FindPackageShare(package='three_legged_robot').find('three_legged_robot')
    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
                launch_arguments=[("gz_args", [" empty.sdf" ])])
    urdfModelPath = os.path.join(pkgPath, 'urdf/three_legged.urdf.xacro')
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('three_legged_robot'),
            'config',
            'controller.yaml',
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

    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robot'],
        output='screen'
    )

    rqt_reconfigure_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller',
                   '--controller-manager',
                   '/controller_manager'],
        output='screen'
    )

    # wheel_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['wheel_controller',
    #                '--controller-manager',
    #                '/controller_manager'],
    #     output='screen'
    # )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[leg_controller_spawner,
                        # wheel_controller_spawner
                        ],
            )
        ),
        bridge,
        robot_state_publisher_node,
        robot_spawn_node,
        rqt_reconfigure_node,
        joint_state_publisher_node,
        start_rviz_cmd
    ])




 world_path = os.path.join(pkg_path, 'worlds', 'mad_world.sdf')