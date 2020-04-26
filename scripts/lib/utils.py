import path
import re
import os
import subprocess as sp
import yaml
import sys
import json

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
    try:
        ros_package_path = os.environ['ROS_PACKAGE_PATH']
    except KeyError:
        exit_error('Set ROS_PACKAGE_PATH correctly')

    paths = ros_package_path.split(':')

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
    try:
        ros_package_path = os.environ['ROS_PACKAGE_PATH']
    except KeyError:
        exit_error('Set ROS_PACKAGE_PATH correctly')

    paths = ros_package_path.split(':')

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

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_error(err_msg):
    print(bcolors.FAIL + err_msg + bcolors.ENDC)

def print_success(msg):
    print(bcolors.OKGREEN + msg + bcolors.ENDC)

def print_debug(msg):
    print(bcolors.HEADER + msg + bcolors.ENDC)

def exit_error(err_msg):
    sys.exit(bcolors.FAIL + err_msg + bcolors.ENDC)

def config_file_path(args, cwd):
    if len(args) > 1:
        file_name = args[1]
        if not os.path.isfile(os.path.join(cwd, file_name)):
            exit_error('The specified configuration file does not exist.')
        if os.path.isfile(file_name): # Handling absolute path
            return file_name
    else:
        file_name = 'rowma.yml'

    return os.path.join(cwd, file_name)

def _load_config(yaml_path):
    config = {}
    if os.path.isfile(yaml_path):
        stream = open(yaml_path, 'r')
        config = yaml.safe_load(stream)
    return config

def get_subscribers_from_yaml(yaml_path):
    config = _load_config(yaml_path)
    subscribers = config.get('topic_destinations')
    if subscribers: print_success("Subscribers were loaded.\n" + json.dumps(subscribers))
    return subscribers

def get_fluentd_stream_topics(yaml_path):
    config = _load_config(yaml_path)
    stream_topics = config.get('fluentd_stream_topics')
    if stream_topics: print_success("Fluentd stream topics were loaded.\n" + json.dumps(stream_topics))
    return stream_topics

def get_file_outputs(yaml_path):
    config = _load_config(yaml_path)
    file_outputs = config.get('file_outputs')
    if file_outputs: print_success("File outputs were loaded.\n" + json.dumps(file_outputs))
    return file_outputs
