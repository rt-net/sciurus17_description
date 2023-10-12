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
        self.use_gazebo = 'false'

    def load(self):
        return Command([
                'xacro ',
                self.robot_description_path,
                ' use_gazebo:=', self.use_gazebo
                ])
