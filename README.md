## rowma_ros (kinetic) - 0.0.2-3

The packages in the `rowma_ros` repository were released into the `kinetic` distro by running `/usr/bin/bloom-release --rosdistro kinetic --track kinetic rowma_ros` on `Sun, 12 Apr 2020 13:35:55 -0000`

The `rowma_ros` package was released.

Version of package(s) in repository `rowma_ros`:

- upstream repository: https://github.com/rowma/rowma_ros
- release repository: unknown
- rosdistro version: `null`
- old version: `0.0.2-2`
- new version: `0.0.2-3`

Versions of tools used:

- bloom version: `0.9.3`
- catkin_pkg version: `0.4.16`
- rosdep version: `0.18.0`
- rosdistro version: `0.8.0`
- vcstools version: `0.1.42`


## rowma_ros (melodic) - 0.0.2-1

The packages in the `rowma_ros` repository were released into the `melodic` distro by running `/usr/bin/bloom-release --rosdistro melodic --track melodic rowma_ros` on `Sun, 12 Apr 2020 12:41:08 -0000`

The `rowma_ros` package was released.

Version of package(s) in repository `rowma_ros`:

- upstream repository: https://github.com/rowma/rowma_ros
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `0.0.2-1`

Versions of tools used:

- bloom version: `0.9.3`
- catkin_pkg version: `0.4.16`
- rosdep version: `0.18.0`
- rosdistro version: `0.8.0`
- vcstools version: `0.1.42`


## rowma_ros (kinetic) - 0.0.2-2

The packages in the `rowma_ros` repository were released into the `kinetic` distro by running `/usr/bin/bloom-release --rosdistro kinetic --track kinetic rowma_ros` on `Sun, 12 Apr 2020 10:33:49 -0000`

The `rowma_ros` package was released.

Version of package(s) in repository `rowma_ros`:

- upstream repository: https://github.com/rowma/rowma_ros
- release repository: unknown
- rosdistro version: `null`
- old version: `0.0.2-1`
- new version: `0.0.2-2`

Versions of tools used:

- bloom version: `0.9.3`
- catkin_pkg version: `0.4.16`
- rosdep version: `0.18.0`
- rosdistro version: `0.8.0`
- vcstools version: `0.1.42`


## rowma_ros (kinetic) - 0.0.2-1

The packages in the `rowma_ros` repository were released into the `kinetic` distro by running `/usr/bin/bloom-release --rosdistro kinetic --track kinetic rowma_ros` on `Sun, 12 Apr 2020 10:19:23 -0000`

The `rowma_ros` package was released.

Version of package(s) in repository `rowma_ros`:

- upstream repository: https://github.com/rowma/rowma_ros
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `0.0.2-1`

Versions of tools used:

- bloom version: `0.9.3`
- catkin_pkg version: `0.4.16`
- rosdep version: `0.18.0`
- rosdistro version: `0.8.0`
- vcstools version: `0.1.42`


# rowma ROS
[![Build Status](https://travis-ci.com/asmsuechan/rowma_ros.svg?branch=master)](https://travis-ci.com/asmsuechan/rowma_ros)

<p align="center">
  <img width="460" src="/logo.png">
</p>

## About
This package is responsible for **WebSocket** connection and is a WebSocket client to operate a ros-based robot.

### rowma repository
Check [the rowma main repository](https://github.com/asmsuechan/rowma) for more information to utilize rowma.

## Requirements
* Python2.7
* ROS Kinetic

Melodic is also tested. Need more tests.

## Usage
Just run `rosrun rowma_ros rowma` and you will get UUID when it successfully connects to the rowma server. You can specify your own rowma server though, the default rowma server is provided at `18.176.1.219`.

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
You can run your own rowma server (its name is `rowma_connection_server`) to avoid connecting your robot to the public server.

```sh
git clone https://github.com/asmsuechan/rowma_connection_manager
cd rowma_connection_manager
npm i
npm run build
sudo npm start
```

And specify the server address by `ROWMA_SERVER_URL` on starting.

```sh
ROWMA_SERVER_URL=http://localhost rosrun rowma_ros rowma
```

For more information about `rowma_connection_manager`, check [the repository](https://github.com/asmsuechan/rowma_connection_manager).

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
## TODO
* versioning
* more tests
