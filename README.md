# rowma ROS
[![Build Status](https://travis-ci.com/asmsuechan/rowma_ros.svg?branch=master)](https://travis-ci.com/asmsuechan/rowma_ros)

<p align="center">
  <img width="460" src="/logo.png">
</p>

## About
This package is responsible for **WebSocket** connection and is a WebSocket client to operate a ros-based robot.

[[rowma/rowma](https://github.com/rowma/rowma)] [[Document](https://rowma.github.io/documentation/en/rowma-ros-overview)]

## Requirements
* Python2.7
* ROS1

## Install
Run this command, then cli installer starts.

```sh
python <(curl "https://raw.githubusercontent.com/rowma/rowma_ros/master/install.py" -s -N)
```

<details><summary>Manual installation</summary>
<p>
You can install `rowma_ros` manually if you fail the installation by the above command.

```sh
cd ~/catkin_ws/src
git clone https://github.com/rowma/rowma_ros
cd rowma_ros
pip install -r requirements.txt
catkin_make
```
</p>
</details>

## Usage
Just run `rosrun rowma_ros rowma` and you will get UUID when it successfully connects to the rowma server.

```
rosrun rowma_ros rowma
```

**Caution**: your robot will be exposed to the Internet after the rosrun command is successfully executed. For more information, see Other section in this page.

## Options
There are some options that can be specified to `rosrun rowma_ros rowma` through environment variables like this below.

```
ROWMA_SERVER_URL=http://localhost:3000 ROWMA_DEBUG=True UUID=my-uuid rosrun rowma_ros rowma
```

|name|value|default|description|
|:-|:-|:-|:-|
|ROWMA_DEBUG|boolean|False|Debug logs are shown if you set this value as True.|
|API_KEY|string|None|API_KEY can be specified for authentication. This value is used along with an authenticator server specified by ROWMA_SERVER_URL.|
|ROWMA_SERVER_URL|string|https://rowma.moriokalab.com|This value describes ConnectionManager URL.|
|UUID|string|None|You can set an arbitrary UUID by using this variable.|
|ROWMA_FLUENTD_ENABLED|boolean|False|We support fluentd to emit rostopic  data. Set True to enable fluentd emission.|
|ROWMA_FLUENTD_HOST|string|localhost|You can specify your fluentd's host name.|
|ROWMA_FLUENTD_PORT|number|24224|You can specify your fluentd's port number.|
|ENABLE_SCRIPT_DOWNLOAD|boolean|False|Any python code will be downloaded and executed as a ROS node in `rowma directory/scripts` if you set this value True.|

## With Fluentd
`rowma_ros` emits rostopic messages which is spcecified in `rowma.yml` to fluentd.

(Write Later)

```
docker run -it --rm -p 24224:24224 -p 24224:24224/udp -v `pwd`:/fluentd/etc fluentd -c /fluentd/etc/td-agent.conf -v
```

## Configuration file (rowma.yml)
You can specify topic destinations to other robots or applications by using rowma.yml.

By default, rowma ros tries to find `rowma.yml` from current directory and use it if exists.

Run rosrun command with file path (both absolute path and relative path are supported) if you clearly specify the file.

```sh
rosrun rowma_ros rowma ./rowma.yml
```

The file has to be this format below:

```yaml
topic_destinations:
  - destination:
      type: robot
      uuid: abc-abc*
    topic: /chatter
  - destination:
      type: robot
      uuid: abc-def*
    topic: /test

fluentd_stream_topics:
  - topic: /piyo

file_outputs:
  - topic: /chatter
    filepath: ./topiclog
```

This configuration says that your robot sends received /topic1 to uuid-of-robot-a and /topic2 to uuid-of-robot-b.

## Development
You can use Docker container when you develop this package.

```sh
# using Docker
docker build -t rowma_ros -f Dockerfile .
docker run --rm --network="host" -e ROWMA_SERVER_URL=http://127.0.0.1 -it rowma_ros
```

## Testing
This ros package is tested on docker images because some functions in `lib.utils` depend on ROS related directory especially `ROS_PACKAGE_PATH`. (`Docker >= 18.09.6`)

```sh
docker build -t rowma_ros_melodic_test -f Dockerfile.test.kinetic .
docker build -t rowma_ros_melodic_test -f Dockerfile.test.melodic .
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_kinetic_test
docker run --rm -v `pwd`:/root/my_workspace/src/rowma_ros -it rowma_ros_melodic_test
```

## Other
You can specify your own rowma server though, the default rowma server is provided at `https://rowma.moriokalab.com`.

### default server
The default rowma server is located at `https://rowma.moriokalab.com` as explained above. It is a **public server** without any auth, therefore you should be careful to connect your robot because anyone can run your robot easily. I recommend you to use the default server for experiments.

Do you unhappy your robots to be exposed? See [Rowma.io](https://rowma.io).

### your own rowma server
You can run your own rowma server (its name is ConnectionManager) to avoid connecting your robot to the public server ([official document](https://rowma.github.io/documentation/en/host-your-network)).

In addition, specify the server address by `ROWMA_SERVER_URL` when you run the node.

```sh
ROWMA_SERVER_URL=http://localhost:3000 rosrun rowma_ros rowma
```

For more information about ConnectionManager, check [the repository](https://github.com/rowma/rowma).

## License
The GPL License (GPL) 2020 - rowma project. Please have a look at the LICENSE.md for more details.
