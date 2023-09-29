# Copyright 2023 RT Corporation

from ament_index_python.packages import get_package_share_directory
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': description_loader.load()}])
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen')

    rviz_config_file = get_package_share_directory(
        'sciurus17_description') + '/launch/display.rviz'
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file])

    ld = LaunchDescription()
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(rviz)

    return ld
