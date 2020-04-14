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
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    paths = ros_package_path.split(':')

    if len(paths) < 1:
        sys.exit('Set ROS_PACKAGE_PATH correctly')

    package_paths = []
    for path in paths:
        m = re.match(r'^/opt/ros', path)
        if m:
            break
        package_paths.append(path)

    return package_paths

def install_path():
    path = get_ros_package_path()
    install_path = ""

    if len(path) == 1:
        install_path_confirmation = input_function("Do you really install at " + bcolors.WARNING + "\"" + path[0]  + "\""+ bcolors.ENDC + " [Y/n] ") or "Y"
        if install_path_confirmation == "Y":
            install_path = path[0]
        elif install_path_confirmation == "n":
            install_path = input_function("[Input install path] ")
        else:
            exit_error("Invalid input, aborted.")
    else:
        print("Multiple package paths are specified.")
        for p in path:
            print("\"" + p + "\"")
        install_path = input_function("[Input install path] ")

    if len(install_path) == 0:
        exit_error("You have to specify 1 install path, aborted.")
    return install_path

def clone(install_path):
    if os.path.exists(install_path + "/rowma_ros"):
        exit_error("You already have rowma_ros package, aborted.")
    clone_args = ["clone", "https://github.com/rowma/rowma_ros.git", install_path + "/rowma_ros"]
    return subprocess.check_call(["git"] + list(clone_args))

def pip_install(install_path):
    pip_args = ["install", "-r", install_path + "/requirements.txt"]
    return subprocess.check_call(["pip"] + list(pip_args))

def catkin_make(install_path):
    return subprocess.check_call(["catkin_make"])

def install():
    try:
        path = install_path()
        clone(path)
        print_success("rowma_ros was successfuly downloaded at " + path)
        pip_install(path)
        print_success("Dependent packages are installed successfuly.")
        catkin_make(path)
        print_success("rowma_ros was successfuly installed!")
    except KeyboardInterrupt:
        exit_error("\nUser interruptation occurred.")

install()
