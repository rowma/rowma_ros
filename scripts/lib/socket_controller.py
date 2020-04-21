import os
import ast
import time
import json
import sys
from subprocess import Popen, PIPE
import rosnode
import rospkg

from lib import utils

class SocketController:
    def __init__(self, id, subscribers, sio, nms):
        self.id = id
        self.subscribers = subscribers
        self.sio = sio
        self.nms = nms
        self.reconnecting = False

    def connect(self):
        ros_root = rospkg.get_ros_root()
        r = rospkg.RosPack()
        manifest = r.get_manifest('rowma_ros')
        rowma_ros_version = manifest.version

        launch_commands = utils.list_launch_commands()
        rosrun_commands = utils.list_rosorun_commands()
        rostopics = utils.list_rostopics()
        uuid = os.environ.get('UUID') or self.id
        msg = {
                'uuid': uuid,
                'launch_commands': launch_commands,
                'rosnodes': rosnode.get_node_names(),
                'rosrun_commands': rosrun_commands,
                'rowma_ros_version': rowma_ros_version,
                'rostopics': rostopics,
                'reconnection': self.reconnecting
                }

        api_key = os.environ.get('API_KEY')
        if api_key:
            msg['api_key'] = api_key

        self.sio.emit('register_robot', json.dumps(msg), namespace=self.nms)
        utils.print_success('connection established')

    def robot_registered(self, data):
        self.id = self.id or data['uuid']
        utils.print_success('Your UUID is: ' + self.id)

    def rostopic(self, data, protocol):
        # TODO: Separate by operation
        if data['op'] == 'subscribe':
            newSubscriber = { 'topic': data['topic'], 'destination': data['topicDestination'] }
            self.subscribers.append(newSubscriber)
        message = ast.literal_eval(json.dumps(data))
        protocol.incoming(json.dumps(message))

    def run_launch(self, data):
        launch_commands = utils.list_launch_commands()
        if data.get('command') in launch_commands:
            cmd = 'PYTHONUNBUFFERED=false roslaunch ' + data.get('command')
            process = Popen(cmd, shell=True, stdout=PIPE)

            # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
            # Therefore, sleep is neccessary to wait it finishes to launch.
            time.sleep(2)
            msg = {
                'uuid': self.id,
                'rosnodes': rosnode.get_node_names(),
                'rostopics': utils.list_rostopics()
                }
            self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    msg = {
                            "cmd": data.get('command').replace(" ", "-"),
                            "robotUuid": self.id,
                            "log": output.strip(),
                          }
                    self.sio.emit('roslaunch_log', json.dumps(msg), namespace=self.nms)
                    print(output.strip())
            process.poll()

    def run_rosrun(self, data):
    	rosrun_commands = utils.list_rosorun_commands()
    	if data.get('command') in rosrun_commands:
    	    cmd = 'PYTHONUNBUFFERED=false rosrun ' + data.get('command') + ' ' + data.get('args')
            process = Popen(cmd, shell=True, stdout=PIPE)

    	    # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
    	    # Therefore, sleep is neccessary to wait it finishes to launch.
    	    time.sleep(2)
    	    msg = {
    	        'uuid': self.id,
    	        'rosnodes': rosnode.get_node_names(),
                'rostopics': utils.list_rostopics()
    	        }
    	    self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    msg = {
                            "cmd": data.get('command').replace(" ", "-"),
                            "robotUuid": self.id,
                            "log": output.strip(),
                          }
                    self.sio.emit('rosrun_log', json.dumps(msg), namespace=self.nms)
                    print(output.strip())
            process.poll()

    def kill_rosnodes(self, data):
        rosnode.kill_nodes(data.get('rosnodes'))
        # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
        # Therefore, sleep is neccessary to wait it finishes to launch.
        time.sleep(2)
        msg = {
            'uuid': self.id,
            'rosnodes': rosnode.get_node_names(),
            'rostopics': utils.list_rostopics()
            }
        self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
        print('killed')

    def unsubscribe_rostopic(self, data):
        utils.print_debug(data)
        self.subscribers = list(filter(lambda s: s['topic'] != data.get('topic'), self.subscribers))
    	msg = {
    	    'uuid': self.id,
    	    'rosnodes': rosnode.get_node_names(),
            'rostopics': utils.list_rostopics()
    	    }
    	self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

    def add_script(self, data):
        enable_script_download = os.getenv('ENABLE_SCRIPT_DOWNLOAD')
        if enable_script_download:
            script = data.get('script')
            name = data.get('name')
            current_path = os.path.dirname(os.path.realpath(__file__))
            file_path = os.path.join(current_path, "../" + name)
            file = open(file_path, 'w')
            file.write(script)
            file.close()

            cmd = 'chmod +x ' + file_path
            process = Popen(cmd, shell=True, stdout=PIPE)

    	    msg = {
    	        'uuid': self.id,
                'rosrunCommands': utils.list_rosorun_commands()
    	        }
    	    self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
        else:
            utils.print_error("You have to set ENABLE_SCRIPT_DOWNLOAD if you use script download.")

    def signal_handler(self):
        self.sio.disconnect()
        sys.exit(0)

    def outgoing_func(self, message):
        if len(self.subscribers) == 0:
            return
        destinations = []
        msg = json.loads(message)
        for subscriber in self.subscribers:
            if subscriber['topic'] == msg['topic']:
                destination = subscriber['destination']

        if destination:
            msg['topicDestination'] = destination
            msg['sourceUuid'] = self.id
            self.sio.emit('topic_from_ros', json.dumps(msg), namespace=self.nms)

    def set_reconnecting(self, reconnecting):
        self.reconnecting = reconnecting
