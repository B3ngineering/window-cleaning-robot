import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package directories
    pkg_name = 'pulley_package'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'cable_robot.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'cable_control.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'cable_world.world')
    
    # Process URDF with xacro
    robot_description_content = xacro.process_file(urdf_file).toxml()
    # Replace $(find ...) with actual path for Gazebo plugin
    robot_description_content = robot_description_content.replace(
        '$(find pulley_package)', pkg_share
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Gazebo launch with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'extra_gazebo_args': '--ros-args --params-file ' + controller_config
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Spawn robot - platform starts at center of wall
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'cable_robot',
            '-x', '0', '-y', '0.2', '-z', '1.5',  # Center of wall
        ],
        output='screen'
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # Motor Velocity Controller
    motor_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motor_velocity_controller', '--controller-manager', '/controller_manager'],
    )
    
    # Platform Effort Controller
    platform_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['platform_effort_controller', '--controller-manager', '/controller_manager'],
    )
    
    # Cable Physics Simulator (delayed start)
    cable_physics = Node(
        package=pkg_name,
        executable='cable_physics',
        output='screen',
    )
    
    # Delay controller spawner until robot is spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_motor_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_velocity_controller_spawner],
        )
    )
    
    # Delay platform effort controller
    delay_platform_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=motor_velocity_controller_spawner,
            on_exit=[platform_effort_controller_spawner],
        )
    )
    
    # Start cable physics after controllers are up
    delay_cable_physics = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=platform_effort_controller_spawner,
            on_exit=[TimerAction(period=2.0, actions=[cable_physics])],
        )
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_motor_controller,
        delay_platform_controller,
        delay_cable_physics,
    ])
