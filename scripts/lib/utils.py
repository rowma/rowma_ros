import path
import re
import os
import subprocess as sp

# Description: Shape path to roslaunch available command.
# @param: path <string> '/dir/package_name/launch/package.launch'
# output: command <string> 'package_name package.launch'
def path_to_command(path):
    splited_path = path.split('/')
    if (len(splited_path) < 3):
        return
    i = 0
    launch_index = 0
    while (i < len(splited_path)):
        if (splited_path[i] == 'launch'):
            launch_index = i
        i += 1
    return splited_path[launch_index - 1] + ' ' + splited_path[launch_index + 1]

# /src/ or /devel/lib/
def path_to_rosrun_command(path):
    splited_path = path.split('/')
    if (len(splited_path) < 3): return
    i = 0
    result = ''
    while (i < len(splited_path)):
        if (splited_path[i] == 'scripts'):
            result = splited_path[i - 1] + ' ' + splited_path[i + 1]
        elif (splited_path[i] == 'lib'):
            # To deal with ~/catkin_ws/devel/lib/some_lib.so
            if (i + 2 >= len(splited_path)): break
            result = splited_path[i + 1] + ' ' + splited_path[i + 2]
        i += 1
    return result

# Description: Get package commands list located at ROS_PACKAGE_PATH environment variable
# output: commands Array<string> ['package_name package.launch']
def list_launch_commands():
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    paths = ros_package_path.split(':')

    if len(paths) < 1:
        sys.exit('Set ROS_PACKAGE_PATH correctly')

    packages = []
    for path in paths:
        m = re.match(r'^/opt/ros', path)
        if m:
            break
        packages += sp.check_output("find " + path + " | grep \'\\.launch\'", shell=True).decode('utf-8').strip().split('\n')
    commands = []
    for package_path in packages:
        commands.append(path_to_command(package_path))
    return commands

def list_rosorun_commands():
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    paths = ros_package_path.split(':')

    if len(paths) < 1:
        sys.exit('Set ROS_PACKAGE_PATH correctly')

    packages = []
    for path in paths:
        m = re.match(r'^/opt/ros', path)
        if m:
            break
        # Cut the last dir path
        ws_root = re.sub('[^/]+(?=/$|$)', '', path)
        packages += sp.check_output("find " + ws_root + " -maxdepth 4 -perm -111 -type f | grep -E \'devel|src\'", shell=True).decode('utf-8').strip().split('\n')

    commands = []
    for package_path in packages:
        commands.append(path_to_rosrun_command(package_path))

    commands = list(filter(None, commands))
    return commands

def list_rostopics():
    rostopics = sp.check_output("rostopic list", shell=True).decode('utf-8').strip().split('\n')
    return rostopics
