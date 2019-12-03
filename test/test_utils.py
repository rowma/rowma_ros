import pytest

import sys
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(1, parentdir + '/scripts')
# TODO: Need more sophisticated import
from lib import utils

def test_path_to_command():
    package_path = "/dir/package_name/launch/package.launch"
    command = utils.path_to_command(package_path)
    assert command == 'package_name package.launch'

def test_path_to_rosrun_command():
    package_path = "/root/catkin_ws/src/package_name/scripts/script"
    command = utils.path_to_rosrun_command(package_path)
    assert command == 'package_name script'

def test_list_launch_commands():
    launch_commands = utils.list_launch_commands()
    assert launch_commands == ['ros_package_template ros_package_template.launch', 'ros_package_template ros_package_template_overlying_params.launch']

def test_list_rosrun_commands():
    rosrun_commands = utils.list_rosorun_commands()
    assert rosrun_commands == ['rowma_ros rowma', 'ros_package_template ros_package_template']

if __name__ == '__main__':
    pytest.main(['-v', __file__])
