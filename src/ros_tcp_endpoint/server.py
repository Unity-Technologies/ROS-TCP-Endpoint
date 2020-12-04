#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rospy
import socket
import json
import sys

from .unity_ip_sender import UnityIPSender
from .client import ClientThread
from .subscriber import RosSubscriber
from .publisher import RosPublisher
from ros_tcp_endpoint.msg import RosUnitySysCommand

class TCPServer:
    """
    Initializes ROS node and TCP server.
    """

    def __init__(self, node_name, buffer_size=1024, connections=10):
        """
        Initializes ROS node and class variables.

        Args:
            node_name:               ROS node name for executing code
            buffer_size:             The read buffer size used when reading from a socket
            connections:             Max number of queued connections. See Python Socket documentation
        """
        self.tcp_ip = rospy.get_param("/ROS_IP")
        self.tcp_port = rospy.get_param("/ROS_TCP_PORT", 10000)

        unity_machine_ip = rospy.get_param("/UNITY_IP", '')
        unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT", 5005)
        self.unity_tcp_sender = UnityIPSender(unity_machine_ip, unity_machine_port)

        self.node_name = node_name
        self.source_destination_dict = {}
        self.buffer_size = buffer_size
        self.connections = connections
        self.syscommands = SysCommands(self)

    def start(self):
        """
            Creates and binds sockets using TCP variables then listens for incoming connections.
            For each new connection a client thread will be created to handle communication.
        """
        rospy.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.tcp_ip, self.tcp_port))
        threads = []

        while True:
            tcp_server.listen(self.connections)

            (conn, (ip, port)) = tcp_server.accept()
            new_thread = ClientThread(conn, self, ip, port)
            new_thread.start()
            threads.append(new_thread)

        # Unreachable statements:
        # for t in threads:
        #    t.join()

    def send_unity_error(self, error):
        self.unity_tcp_sender.send_unity_error(error)

    def send_unity_message(self, topic, message):
        self.unity_tcp_sender.send_unity_message(topic, message)

    def handle_syscommand(self, data):
        message = RosUnitySysCommand().deserialize(data)
        function = getattr(self.syscommands, message.command)
        if function is None:
            self.send_unity_error("Don't understand SysCommand.'{}'({})".format(message.command, message.params_json))
            return
        else:
            params = json.loads(message.params_json)
            function(**params)


class SysCommands:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server

    def subscribe(self, topic, message_name):
        if topic == '':
            self.tcp_server.send_unity_error(
                "Can't subscribe to a blank topic name! SysCommand.subscribe({}, {})".format(topic, message_name))
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error("SysCommand.subscribe - Unknown message class {}".format(message_name))
            return

        print("RegisterSubscriber({}, {}) OK".format(topic, message_class))
        self.tcp_server.source_destination_dict[topic] = RosSubscriber(topic, message_class, self.tcp_server)

    def publish(self, topic, message_name):
        if topic == '':
            self.tcp_server.send_unity_error(
                "Can't publish to a blank topic name! SysCommand.publish({}, {})".format(topic, message_name))
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error("SysCommand.publish - Unknown message class {}".format(message_name))
            return

        print("RegisterPublisher({}, {}) OK".format(topic, message_class))
        self.tcp_server.source_destination_dict[topic] = RosPublisher(topic, message_class, queue_size=10)


def resolve_message_name(name):
    try:
        names = name.split('/')
        module = sys.modules[names[0]]
        if module is None:
            print("Failed to resolve module", names[0])
        module = getattr(module, 'msg')
        if module is None:
            print("Failed to resolve module.msg", names[0])
        module = getattr(module, names[1])
        if module is None:
            print("Failed to resolve {}.msg.{}".format(names[0], names[1]))
        return module
    except (KeyError, AttributeError) as e:
        return None
