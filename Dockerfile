FROM ros:melodic

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install python2.7 python-pip python3-pip -y && \
    pip install --upgrade pip
RUN apt-get install -y ros-melodic-rosbridge-server

COPY . /root/my_workspace/src/rowma_ros

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/melodic/setup.bash && \
    catkin_init_workspace && \
    git clone https://github.com/leggedrobotics/ros_best_practices /tmp/ros_best_practices && \
    cd /tmp/ros_best_practices && \
    git checkout dcc45f537ec9876fcef318fcae435e93fcaf71e1 && \
    mv -f /tmp/ros_best_practices/ros_package_template /root/my_workspace/src/ && \
    cd /root/my_workspace/src/rowma_ros && \
    pip2 install -r requirements.txt && \
    pip2 install path.py pytest && \
    cd /root/my_workspace && \
    catkin_make

CMD ["/bin/bash", "-c", "(roscore &) && source /root/my_workspace/devel/setup.bash && rosrun rowma_ros rowma"]
