# rowma ROS
[![Build Status](https://travis-ci.com/asmsuechan/rowma_ros.svg?branch=master)](https://travis-ci.com/asmsuechan/rowma_ros)

## About
This package **does not** Publish/Subscribe any rostopics. This package is responsible for **WebSocket** connection and is a WebSocket client to operate a ros-based robot.

### rowma repository
Check [the rowma main repository](https://github.com/asmsuechan/rowma) for more information to utilize rowma.

## Requirements

## Usage

```sh
rosrun rowma_ros rowma
ROWMA_SERVER_URL=http://localhost rosrun rowma_ros rowma
```

## Testing
This ros package is tested on docker images because some functions in `lib.utils` depend on ROS related directory especially `ROS_PACKAGE_PATH`. (`Docker >= 18.09.6`)

```sh
docker build -t rowma_ros_melodic_test -f Dockerfile.test.kinetic .
docker build -t rowma_ros_melodic_test -f Dockerfile.test.melodic .
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_kinetic_test
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_melodic_test
```

## Development
```sh
# using Docker
docker build -t rowma_ros -f Dockerfile .
docker run --rm --network="host" -e ROWMA_SERVER_URL=http://127.0.0.1 -it rowma_ros
```
