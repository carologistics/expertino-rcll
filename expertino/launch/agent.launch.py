import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_with_context(context, *args, **kwargs):
    expertino_dir = get_package_share_directory('expertino')
    manager_config = LaunchConfiguration("manager_config")
    log_level = LaunchConfiguration('log_level')
    manager_config_file = os.path.join(expertino_dir, "params", manager_config.perform(context))
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(expertino_dir + "/clips/domain.pddl"),
        description='PDDL Model file')

    # also launch the pddl_manager
    pddl_manager_dir = get_package_share_directory('pddl_manager')
    launch_pddl_manager = os.path.join(pddl_manager_dir, 'launch', 'pddl_manager.launch.py')

    cx_node = Node(
        package='cx_bringup',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            manager_config_file,
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    return [cx_node, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_pddl_manager)
        ),]

def generate_launch_description():

    model_file = LaunchConfiguration('model_file')


    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value='info',
        description="Logging level for cx_node executable",
    )
    declare_manager_config = DeclareLaunchArgument(
        "manager_config",
        default_value="clips_env_manager.yaml",
        description="Name of the CLIPS environment manager configuration",
    )

    refbox_node = Node(
            package='expertino',
            executable='refbox.py',
            name='refbox_node',
            output='screen',  # Show the output in the terminal
            emulate_tty=True,  # Emulate a terminal to support interactive commands
            parameters=[
                {'terminal': 'gnome-terminal'},
                {'shell': 'bash'}
            ],
        )

    # plansys2_node_cmd = Node(
    #     package='cx_bringup',
    #     executable='plansys_node',
    #     output='screen',
    #     parameters=[
    #         {
    #             'model_file': model_file,
    #         },
    #         clips_features_manager_file,
    #     ])

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)
    ld.add_action(declare_manager_config)
    # ld.add_action(plansys2_node_cmd)

    ld.add_action(OpaqueFunction(function=launch_with_context))
    #ld.add_action(refbox_node)

    return ld
