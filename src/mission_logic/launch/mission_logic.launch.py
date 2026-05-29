from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_magnetic_field_node = Node(
        package='mission_logic',
        executable='magnetic_field_node',
        name='magnetic_field_node',
        output='screen',
    )

    start_mission_node = Node(
        package='mission_logic',
        executable='mission_node',
        name='mission_node',
        output='screen',
    )

    return LaunchDescription([
        start_magnetic_field_node,
        start_mission_node,
    ])
