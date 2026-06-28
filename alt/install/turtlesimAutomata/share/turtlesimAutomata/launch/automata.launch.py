"""
Launch file for the turtlesim automata assignment.

This launch file starts the turtlesim window and the automata controller node.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the ROS2 launch description."""

    return LaunchDescription([

        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim",
            output="screen"
        ),

        Node(
            package="turtlesimAutomata",
            executable="automata_node",
            name="turtlesim_automata",
            output="screen",
            emulate_tty=True
        )
    ])