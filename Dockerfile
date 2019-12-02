FROM ros:kinetic-robot

RUN apt-get update && \
    apt-get install python2.7 python-pip -y && \
    pip install --upgrade pip

COPY . /root/my_workspace/src/rowma_ros

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    catkin_init_workspace && \
    git clone https://github.com/leggedrobotics/ros_best_practices /tmp/ros_best_practices && \
    mv -f /tmp/ros_best_practices/ros_package_template /root/my_workspace/src/ && \
    cd /root/my_workspace/src/rowma_ros && \
    pip install -r requirements.txt && \
    cd /root/my_workspace && \
    catkin_make

CMD ["python", "test/test_utils.py"]
