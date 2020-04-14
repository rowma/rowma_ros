# rowma ROS
[![Build Status](https://travis-ci.com/asmsuechan/rowma_ros.svg?branch=master)](https://travis-ci.com/asmsuechan/rowma_ros)

<p align="center">
  <img width="460" src="/logo.png">
</p>

## About
This package is responsible for **WebSocket** connection and is a WebSocket client to operate a ros-based robot.

### rowma repository
Check [the rowma main repository](https://github.com/rowma/rowma) for more information to utilize rowma.

## Requirements
* Python2.7
* ROS1

## Usage
Just run `rosrun rowma_ros rowma` and you will get UUID when it successfully connects to the rowma server. You can specify your own rowma server though, the default rowma server is provided at `18.176.1.219`.

```sh
python <(curl "https://raw.githubusercontent.com/rowma/rowma_ros/master/install.py" -s -N)
```

### Manual installation
You can install `rowma_ros` manually if you fail the installation from `rowma/rowma_ros/install.py`.

```sh
cd ~/catkin_ws/src
git clone https://github.com/asmsuechan/rowma_ros
cd rowma_ros
pip install -r requirements.txt
catkin_make
rosrun rowma_ros rowma
```

### default server
The default rowma server is provided at `18.176.1.219` as explained above. It is a **public server** without any auth, therefore you should be careful to connect your robot because anyone can run your robot easily. I recommend you to use the default server for experiments.

### your own rowma server
You can run your own rowma server (its name is `rowma_connection_server`) to avoid connecting your robot to the public server ([official document](https://rowma.github.io/documentation/en/host-your-network)).

In addition, specify the server address by `ROWMA_SERVER_URL` when you run the node.

```sh
ROWMA_SERVER_URL=http://localhost rosrun rowma_ros rowma
```

For more information about ConnectionManager, check [the repository](https://github.com/rowma/rowma).

## Testing
This ros package is tested on docker images because some functions in `lib.utils` depend on ROS related directory especially `ROS_PACKAGE_PATH`. (`Docker >= 18.09.6`)

```sh
docker build -t rowma_ros_melodic_test -f Dockerfile.test.kinetic .
docker build -t rowma_ros_melodic_test -f Dockerfile.test.melodic .
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_kinetic_test
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_melodic_test
```

## Development
You can use Docker container when you develop this package.

```sh
# using Docker
docker build -t rowma_ros -f Dockerfile .
docker run --rm --network="host" -e ROWMA_SERVER_URL=http://127.0.0.1 -it rowma_ros
```
