# Copyright 2023 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


class RobotDescriptionLoader():

    def __init__(self):
        self.robot_description_path = os.path.join(
            get_package_share_directory('sciurus17_description'),
            'urdf',
            'sciurus17.urdf.xacro')
        self.port_name = '/dev/sciurus17spine'
        self.baudrate = '3000000'
        self.timeout_seconds = '1.0'
        self.manipulator_config_file_path = ''
        self.use_gazebo = 'false'
        self.use_gazebo_head_camera = 'false'
        self.use_gazebo_chest_camera = 'false'
        self.gz_control_config_package = ''
        self.gz_control_config_file_path = ''

    def load(self):
        return Command([
                'xacro ',
                self.robot_description_path,
                ' port_name:=', self.port_name,
                ' baudrate:=', self.baudrate,
                ' timeout_seconds:=', self.timeout_seconds,
                ' manipulator_config_file_path:=', self.manipulator_config_file_path,
                ' use_gazebo:=', self.use_gazebo,
                ' use_gazebo_head_camera:=', self.use_gazebo_head_camera,
                ' use_gazebo_chest_camera:=', self.use_gazebo_chest_camera,
                ' gz_control_config_package:=', self.gz_control_config_package,
                ' gz_control_config_file_path:=', self.gz_control_config_file_path
                ])
