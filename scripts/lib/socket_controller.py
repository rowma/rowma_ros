import os
import ast
import time
import json
import sys
from subprocess import Popen
import rosnode
import rospkg

from lib import utils

class SocketController:
    def __init__(self, id, launched_nodes, subscribers, sio, nms):
        self.id = id
        self.launched_nodes = launched_nodes
        self.subscribers = subscribers
        self.sio = sio
        self.nms = nms

    def connect(self):
        print('connection established')
        ros_root = rospkg.get_ros_root()
        r = rospkg.RosPack()
        manifest = r.get_manifest('rowma_ros')
        rowma_ros_version = manifest.version

        launch_commands = utils.list_launch_commands()
        rosrun_commands = utils.list_rosorun_commands()
        uuid = os.environ.get('UUID') or self.id
        msg = {
                'uuid': uuid,
                'launch_commands': launch_commands,
                'rosnodes': rosnode.get_node_names(),
                'rosrun_commands': rosrun_commands,
                'rowma_ros_version': rowma_ros_version
                }

        api_key = os.environ.get('API_KEY')
        if api_key:
            msg['api_key'] = api_key

        self.sio.emit('register_robot', json.dumps(msg), namespace=self.nms)

    def robot_registered(self, data):
        self.id = self.id or data['uuid']
        print('Your UUID is: ' + self.id)

    def rostopic(self, data, protocol):
        # TODO: Separate by operation
        if data['op'] == 'subscribe':
            newSubscriber = { 'topic': data['topic'], 'destination': data['topicDestination'] }
            self.subscribers.append(newSubscriber)
        message = ast.literal_eval(json.dumps(data))
        protocol.incoming(json.dumps(message))

    def run_launch(self, data):
        launch_commands = utils.list_launch_commands()
        print(launch_commands)
        if data.get('command') in launch_commands:
            cmd = 'roslaunch ' + data.get('command')
            if self.launched_nodes == None:
                self.launched_nodes =[]
            self.launched_nodes = self.launched_nodes.append(Popen(cmd.split()))

            # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
            # Therefore, sleep is neccessary to wait it finishes to launch.
            time.sleep(2)
            msg = {
                'uuid': self.id,
                'rosnodes': rosnode.get_node_names()
                }
            self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
            print('run_launch')
            print(data)

    def run_rosrun(self, data):
    	rosrun_commands = utils.list_rosorun_commands()
    	print(rosrun_commands)
    	if data.get('command') in rosrun_commands:
    	    cmd = 'rosrun ' + data.get('command') + ' ' + data.get('args')
    	    print(cmd)
    	    self.launched_nodes = self.launched_nodes.append(Popen(cmd.split()) or [])

    	    # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
    	    # Therefore, sleep is neccessary to wait it finishes to launch.
    	    time.sleep(2)
    	    msg = {
    	        'uuid': self.id,
    	        'rosnodes': rosnode.get_node_names()
    	        }
    	    self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
    	    print('run_rosrun')
    	    print(data)

    def kill_rosnodes(self, data):
        rosnode.kill_nodes(data.get('rosnodes'))
        # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
        # Therefore, sleep is neccessary to wait it finishes to launch.
        time.sleep(2)
        msg = {
            'uuid': self.id,
            'rosnodes': rosnode.get_node_names()
            }
        self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
        print('killed')

    def signal_handler(self):
        self.sio.disconnect()
        for node in self.launched_nodes:
            node.terminate()
        sys.exit(0)

    def outgoing_func(self, message):
        destinations = []
        msg = json.loads(message)
        for subscriber in self.subscribers:
            if subscriber['topic'] == msg['topic']:
                destination = subscriber['destination']
        msg['topicDestination'] = destination
        msg['sourceUuid'] = self.id
        self.sio.emit('topic_from_ros', json.dumps(msg), namespace=self.nms)
