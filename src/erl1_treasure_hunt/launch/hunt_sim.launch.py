"""Full treasure hunt in simulation: Gazebo + robot + our 4 nodes."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_erl1 = get_package_share_directory('erl1')
    pkg_hunt = get_package_share_directory('erl1_treasure_hunt')
    params_file = os.path.join(pkg_hunt, 'config', 'sim.yaml')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz')

    # 1. The prof's simulation: world, robot, bridges, EKF, RViz
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_erl1, 'launch', 'spawn_assignment.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items())

    # 2. Our 4 nodes, all reading the same yaml
    params = [params_file, {'use_sim_time': True}]
    perception = Node(package='erl1_perception', executable='aruco_detector',
                      name='aruco_detector', parameters=params, output='screen')
    servoing = Node(package='erl1_treasure_hunt', executable='visual_servoing',
                    name='visual_servoing', parameters=params, output='screen')
    rotation = Node(package='erl1_treasure_hunt', executable='rotation',
                    name='rotation', parameters=params, output='screen')
    state_machine = Node(package='erl1_treasure_hunt', executable='state_machine',
                         name='treasure_hunt', parameters=params, output='screen')

    # 3. Start our nodes only after Gazebo has started and spawned the robot
    hunt = TimerAction(period=10.0,
                       actions=[perception, servoing, rotation, state_machine])

    return LaunchDescription([rviz_arg, simulation, hunt])