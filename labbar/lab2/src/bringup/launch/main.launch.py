#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.utilities import perform_substitutions


def generate_launch_description():
    # --- Paths ---
    bringup_dir = get_package_share_directory('bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')

    # Default paths for files
    default_rviz_config = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    default_nav_params = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_burger.urdf')
    world_file = os.path.join(bringup_dir, 'worlds', 'turtlebot3_world.world')
    map_file = os.path.join(bringup_dir, 'maps', 'map.yaml')
    default_bt_xml = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )
    burger_sdf = os.path.join(bringup_dir,'models', 'turtlebot3_burger', 'model.sdf')

    # Read URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # --- Launch Configurations ---
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=default_rviz_config)
    params_file = LaunchConfiguration('nav_params_file', default=default_nav_params)
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    # --- Launch Arguments Declarations ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', default_value='true', description='Use simulator time'
    )
    declare_enable_rviz_arg = DeclareLaunchArgument(
        name='enable_rviz', default_value='true', description='Enable rviz launch'
    )
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz_config,
        description='Full path to the RVIZ config file to use'
    )
    declare_params_file_arg = DeclareLaunchArgument(
        'nav_params_file', default_value=default_nav_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False', # ----- Sätt till False för att köra, True för att INTE köra
        description='Whether to execute gzclient)')
    # --- Nodes and Launch Includes ---
    # Gazebo
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        cwd=[os.path.join(bringup_dir, 'launch')], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[os.path.join(bringup_dir, 'launch')], output='screen')

    # Common Remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'yaml_filename': map_file}],
        remappings=remappings
    )
    map_server_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'publish_frequency': 10.0,
                     'robot_description': robot_description_content}],
        remappings=remappings,
    )

    # Spawn Turtlebot3 Burger
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', burger_sdf,
            '-entity', 'tb1',
            '-x', '-1.5', '-y', '-0.5', '-z', '0.01', '-Y', '0.0',
            '-unpause',
        ],
        output='screen',
    )

    # Navigation2 Bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'use_namespace': 'False',
            'map': '',  # Using map_server declared above
            'map_server': 'False', # Don't run internal map server
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml,
            'autostart': 'true',
            'use_sim_time': use_sim_time,
            'log_level': 'warn'
        }.items()
    )
    initial_pose_message = (
        '{header: {frame_id: map}, pose: {pose: {position: {x: '
        f'{-1.5}, y: {-0.5}, z: {0.01}'
        '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'
    )
    initial_pose_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', '/initialpose',
             'geometry_msgs/PoseWithCovarianceStamped', initial_pose_message],
        output='screen'
    )

    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_namespace': 'False',
            'rviz_config': rviz_config_file,
            'log_level': 'warn'
        }.items(),
        condition=IfCondition(enable_rviz)
    )

    # --- Event Handlers ---
    # Set initial pose and start RViz after spawning
    post_spawn_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[initial_pose_cmd, rviz_cmd],
        )
    )

    # Launching Obstacle detection Launcher
    obstacle_detection_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('obstacle_detection'), 'launch', 'obstacle_detection.launch.py')
        )
    )

    # --- Add Actions to Launch Description ---
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_enable_rviz_arg)
    ld.add_action(declare_rviz_config_file_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    ld.add_action(map_server_node)
    ld.add_action(map_server_lifecycle_node)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_robot_node)
    ld.add_action(nav2_bringup_cmd)

    ld.add_action(post_spawn_event)
    ld.add_action(obstacle_detection_launch_file)

    return ld
