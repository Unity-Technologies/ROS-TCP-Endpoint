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
import time
import threading
import struct
from .client import ClientThread
from ros_tcp_endpoint.msg import RosUnityError
from ros_tcp_endpoint.srv import UnityHandshake, UnityHandshakeResponse
from queue import Queue


class UnityTcpSender:
    """
    Connects and sends messages to the server on the Unity side.
    """
    def __init__(self, unity_ip, unity_port, timeout):
        self.unity_ip = unity_ip
        self.unity_port = unity_port
        # if we have a valid IP at this point, it was overridden locally so always use that
        self.ip_is_overridden = (self.unity_ip != '')
        self.timeout = timeout
        self.queue = Queue()
        sender_thread = threading.Thread(target=self.sender_loop)
        # Exit the server thread when the main thread terminates
        sender_thread.daemon = True
        sender_thread.start()


    def handshake(self, incoming_ip, data):
        message = UnityHandshake._request_class().deserialize(data)
        self.unity_port = message.port
        if not self.ip_is_overridden:
            # hello Unity, we'll talk to you from now on
            if message.ip == '':
                # if the message doesn't specify an IP, just talk back to the incoming IP address
                self.unity_ip = incoming_ip
            else:
                # otherwise Unity has set an IP override, so use that
                self.unity_ip = message.ip
        print("ROS-Unity Handshake received, will connect to {}:{}".format(self.unity_ip, self.unity_port))
        return UnityHandshakeResponse(self.unity_ip)

    def send_unity_error(self, error):
        self.send_unity_message("__error", RosUnityError(error))

    def send_unity_message(self, topic, message):
        if self.unity_ip == '':
            print("Can't send a message, no defined unity IP!".format(topic, message))
            return

        serialized_message = ClientThread.serialize_message(topic, message)
        self.queue.put(serialized_message)

    def send_unity_service(self, topic, service_class, request):
        if self.unity_ip == '':
            print("Can't send a message, no defined unity IP!".format(topic, request))
            return

        serialized_message = ClientThread.serialize_message(topic, request)
        
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((self.unity_ip, self.unity_port))
            s.sendall(struct.pack('<f', self.timeout))
            s.sendall(serialized_message)

            destination, data = ClientThread.read_message(s)

            response = service_class._response_class().deserialize(data)

            s.close()
            return response
        except Exception as e:
            rospy.loginfo("Exception {}".format(e))
 
    def sender_loop(self):
        s = None
        sockettimeout = 0
        
        while True:
            item = self.queue.get()
            
            if time.time() > sockettimeout:
                if s != None:
                    s.close()

                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(self.timeout)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.connect((self.unity_ip, self.unity_port))
                    s.sendall(struct.pack('<f', self.timeout))
                except Exception as e:
                    rospy.loginfo("Exception on Connect {}".format(e))
                    sockettimeout = 0
            
            try:
                s.sendall(item)
                # on the endpoint, make a new connection after 3/4ths of the timeout time. Trying to avoid
                # a situation where Unity has stopped listening but the endpoint is still trying to send
                sockettimeout = time.time() + self.timeout * 0.75
            except Exception as e:
                rospy.loginfo("Exception on Send {}".format(e))
