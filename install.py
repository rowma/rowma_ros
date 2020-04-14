#!/usr/bin/env python
import os
import sys
import re
import subprocess

global input_function
if sys.version[0] == "2":
    input_function = raw_input
else:
    input_function = input

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def exit_error(err_msg):
    sys.exit(bcolors.FAIL + err_msg + bcolors.ENDC)

def print_success(msg):
    print(bcolors.OKGREEN + msg + bcolors.ENDC)

def get_ros_package_path():
    try:
        ros_package_path = os.environ['ROS_PACKAGE_PATH']
        paths = ros_package_path.split(':')
    except KeyError:
        exit_error('Set ROS_PACKAGE_PATH correctly')

    package_paths = []
    for path in paths:
        m = re.match(r'^/opt/ros', path)
        if m:
            break
        package_paths.append(path)

    return package_paths

def workspace_path():
    path = get_ros_package_path()
    workspace_path = ""

    if len(path) == 1:
        workspace_path_confirmation = input_function("Do you really install at " + bcolors.WARNING + "\"" + path[0]  + "\""+ bcolors.ENDC + " [Y/n] ") or "Y"
        if workspace_path_confirmation == "Y":
            workspace_path = path[0]
        elif workspace_path_confirmation == "n":
            workspace_path = input_function("[Input install path] ")
        else:
            exit_error("Invalid input, aborted.")
    else:
        print("Multiple package paths are specified.")
        for p in path:
            print("\"" + p + "\"")
        workspace_path = input_function("[Input install path] ")

    if len(workspace_path) == 0:
        exit_error("You have to specify 1 install path, aborted.")
    return workspace_path

def clone(workspace_path):
    rowma_ros_path = os.path.join(workspace_path, "/rowma_ros")
    if os.path.exists(rowma_ros_path):
        exit_error("You already have rowma_ros package, aborted.")
    clone_args = ["clone", "https://github.com/rowma/rowma_ros.git", rowma_ros_path]
    return subprocess.check_call(["git"] + list(clone_args))

def pip_install(workspace_path):
    requirements_txt_pat = os.path.join(workspace_path, "/rowma_ros/requirements.txt")
    pip_args = ["install", "-r", requirements_txt_pat]
    return subprocess.check_call(["pip"] + list(pip_args))

def install_apt_deps():
    try:
        distro = os.environ['ROS_DISTRO']
    except KeyError:
        exit_error('Set ROS_DISTRO correctly.')

    return subprocess.check_call(["sudo", "apt-get", "install", "ros-" + distro + "-rosbridge-server"])

def catkin_make(workspace_path):
    working_dir = os.path.join(workspace_path, "/..")
    return subprocess.check_call(["catkin_make"], shell=True, cwd=working_dir)

def install():
    try:
        path = workspace_path()
        clone(path)
        print_success("rowma_ros was successfully downloaded at " + path)
        pip_install(path)
        print_success("Pip dependencies were installed successfully.")
        install_apt_deps()
        print_success("Apt packages were installed successfully.")
        catkin_make(path)
        print_success("rowma_ros was successfully installed!")
    except KeyboardInterrupt:
        exit_error("\nUser interruptation occurred.")

install()
