import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mission_logic'),
        'config',
        'mission_logic.yaml',
    )
    pipeline_config_file = os.path.join(
        get_package_share_directory('mission_logic'),
        'config',
        'straight_wire.json',
    )

    start_magnetic_field_node = Node(
        package='mission_logic',
        executable='magnetic_field_node',
        name='magnetic_field_node',
        output='screen',
        parameters=[
            config_file,
            {'pipeline_config_file': pipeline_config_file},
        ],
    )

    start_mission_node = Node(
        package='mission_logic',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        start_magnetic_field_node,
        start_mission_node,
    ])
