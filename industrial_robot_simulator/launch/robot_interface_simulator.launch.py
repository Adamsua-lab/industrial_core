"""Provides a simulated industrial robot.

Several nodes are started, to simulate low-level robot communication
and higher-level actionlib support:
  - industrial_robot_simulator : accepts motion commands and publishes robot state
  - joint_trajectory_action : actionlib interface to control robot motion

Usage:
  ros2 launch industrial_robot_simulator robot_interface_simulator.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='industrial_robot_simulator',
            executable='industrial_robot_simulator',
            name='industrial_robot_simulator',
        ),
        Node(
            package='industrial_robot_client',
            executable='joint_trajectory_action',
            name='joint_trajectory_action',
        ),
    ])
