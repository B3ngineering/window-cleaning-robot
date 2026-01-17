import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package directories
    pkg_name = 'pulley_package'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'pulley_motor.urdf')
    controller_config = os.path.join(pkg_share, 'config', 'pulley_control.yaml')
    
    # Process URDF with xacro to substitute controller path
    robot_description_content = open(urdf_file).read()
    robot_description_content = robot_description_content.replace(
        '$(arg controller_config_file)', controller_config
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'pulley_system',
                   '-z', '0.5'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
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
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_motor_controller,
    ])