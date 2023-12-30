FROM ros:noetic

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install python3-pip python-is-python3 git -y && \
    pip install --upgrade pip
RUN apt-get install -y python3-colcon-common-extensions ros-noetic-rosbridge-server

COPY . /root/my_workspace/src/rowma_ros

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && \
    # catkin_init_workspace && \
    git clone https://github.com/leggedrobotics/ros_best_practices /tmp/ros_best_practices && \
    cd /tmp/ros_best_practices && \
    git checkout dcc45f537ec9876fcef318fcae435e93fcaf71e1 && \
    mv -f /tmp/ros_best_practices/ros_package_template /root/my_workspace/src/ && \
    cd /root/my_workspace/src/rowma_ros && \
    pip install -r requirements.txt && \
    pip install path.py pytest && \
    cd /root/my_workspace && \
    colcon build

RUN export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/my_workspace/src
RUN roscore &

CMD ["/bin/bash", "-c", "source /root/my_workspace/install/local_setup.bash && rosrun rowma_ros rowma"]
