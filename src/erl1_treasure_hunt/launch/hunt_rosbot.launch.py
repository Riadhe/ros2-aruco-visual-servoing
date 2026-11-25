"""Treasure hunt on the real ROSbot: only our 4 nodes (the robot runs its own driver)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hunt = get_package_share_directory('erl1_treasure_hunt')
    params_arg = DeclareLaunchArgument(
        'params', default_value=os.path.join(pkg_hunt, 'config', 'rosbot.yaml'),
        description='Parameter file for the real robot')

    params = [LaunchConfiguration('params'), {'use_sim_time': False}]

    def node(package, executable, name):
        return Node(package=package, executable=executable, name=name,
                    parameters=params, output='screen')

    return LaunchDescription([
        params_arg,
        node('erl1_perception', 'aruco_detector', 'aruco_detector'),
        node('erl1_treasure_hunt', 'visual_servoing', 'visual_servoing'),
        node('erl1_treasure_hunt', 'rotation', 'rotation'),
        node('erl1_treasure_hunt', 'state_machine', 'treasure_hunt'),
    ])
