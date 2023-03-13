#!/usr/bin/env python
import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from nav2_common.launch import RewrittenYaml
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('sofar_assignment')

    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Parameters
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    keepout_params_file = LaunchConfiguration('keepout_params_file')
    mask_yaml_file = LaunchConfiguration('mask')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    nav2_map = LaunchConfiguration('map', default=os.path.join(package_dir, 'resource', 'map.yaml'))

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    # Declare the launch arguments
    declare_world = DeclareLaunchArgument(
            'world',
            description='Choose one of the world files from `/sofar_assignment/world` directory')

    declare_mode = DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode')

    declare_map = DeclareLaunchArgument(
            'map',
            description='Choose one of the maps from resource directory')
    
    declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')

    declare_keepout_params_file_cmd = DeclareLaunchArgument(
            'keepout_params_file',
            description='Full path to the ROS2 parameters file to use')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
            'mask',
            description='Full path to filter mask yaml file to load')

    # Launch the simulation environment    
    webots = WebotsLauncher(
            world=PathJoinSubstitution([package_dir, 'worlds', world]),
            mode=mode)

    # The Ros2Supervisor node is a special node interacting with the simulation.
    # For example, it publishes the /clock topic of the simulation or permits to spawn robot from URDF files.
    # By default, the respawn option is set to True.
    ros2_supervisor = Ros2SupervisorLauncher()

    webots_handler = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            ))

    # Make re-written yaml
    param_substitutions = {
            'use_sim_time': use_sim_time,
            'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
            source_file=keepout_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Nodes launching commands
    diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout)

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout)

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')]
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['humble', 'rolling']:
        mappings.append(('/diffdrive_controller/odom', '/odom'))

    tiago_driver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Tiago_Iron'},
            parameters=[
                {'robot_description': robot_description,
                 'use_sim_time': use_sim_time,
                 'set_robot_state_publisher': True},
                ros2_control_params
            ],
            remappings=mappings)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>'}])

    footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'])

    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'default.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config=' + rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(use_rviz))

    nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map',nav2_map),
                ('use_sim_time', use_sim_time),
            ])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True, 
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  
            parameters=[configured_params])

    return LaunchDescription([
        declare_world,
        declare_mode,
        declare_map,
        declare_params_file_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_keepout_params_file_cmd,
        declare_mask_yaml_file_cmd,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        rviz,
        webots,
        ros2_supervisor,
        robot_state_publisher,
        tiago_driver,
        footprint_publisher,
        nav2_bringup,
        start_lifecycle_manager_cmd,
        start_map_server_cmd,
        start_costmap_filter_info_server_cmd,
        webots_handler])