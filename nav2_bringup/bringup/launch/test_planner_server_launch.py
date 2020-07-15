import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    lifecycle_nodes = [
        'controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'map_server']

    remappings = []

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    argument_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    argument_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    argument_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    argument_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    argument_default_bt_xml_filename = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_time.xml'),
        description='Full path to the behavior tree xml file to use')

    argument_map_subscribe_transient_local = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    argument_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    argument_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_test_planner_server_view.rviz'),
        description='Full path to the RVIZ config file to use')

    argument_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # External launch files
    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    # Nodes
    node_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    node_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    node_recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    node_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    node_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    node_fake_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    node_fake_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'odom', 'base_link'])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(argument_namespace)
    ld.add_action(argument_use_sim_time)
    ld.add_action(argument_autostart)
    ld.add_action(argument_params_file)
    ld.add_action(argument_default_bt_xml_filename)
    ld.add_action(argument_map_subscribe_transient_local)
    ld.add_action(argument_map)
    ld.add_action(argument_rviz_config_file)
    ld.add_action(argument_use_rviz)

    # Declare external launch files
    ld.add_action(launch_rviz)

    # Declare the nodes
    ld.add_action(node_controller_server)
    ld.add_action(node_planner_server)
    ld.add_action(node_recoveries_server)
    ld.add_action(node_bt_navigator)
    ld.add_action(node_waypoint_follower)
    ld.add_action(node_lifecycle_manager)

    ld.add_action(node_map_server)
    ld.add_action(node_fake_map_to_odom)
    ld.add_action(node_fake_odom_to_base_link)

    return ld
