# Copyright 2023 RT Corporation

from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch.launch_context import LaunchContext
import pytest


def exec_load(loader):
    # Command substitutionの実行方法はCommandのテストを参考にした
    # https://github.com/ros2/launch/blob/074cd2903ddccd61bce8f40a0f58da0b7c200481/launch/test/launch/substitutions/test_command.py#L47
    context = LaunchContext()
    return loader.load().perform(context)


def test_load_description():
    # xacroの読み込みが成功することを期待
    rdl = RobotDescriptionLoader()
    assert exec_load(rdl)


def test_change_description_path():
    # xacroのファイルパスを変更し、読み込みが失敗することを期待
    rdl = RobotDescriptionLoader()
    rdl.robot_description_path = 'hoge'
    with pytest.raises(Exception) as e:
        exec_load(rdl)
    assert e.value


def test_use_gazebo():
    # use_gazeboが変更され、xacroにgazeboタグがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    assert '<gazebo' in exec_load(rdl)
