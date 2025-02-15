from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rcll_sim_path_planner_dir = get_package_share_directory('rcll_sim_path_planner')
    lifecycle_nodes = ['planner_server']

    # Start your node that publishes the occupancy grid
    rcll_map_publisher_node = Node(
        package="rcll_map_publisher",
        executable="rcll_map_publisher",
        name="rcll_map_publisher",
        output="screen"
    )
    nav2_planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[rcll_sim_path_planner_dir  + '/params/navigation.yaml'])
    nav2_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        rcll_map_publisher_node,
        nav2_planner_node,
        nav2_lifecycle_node
    ])
