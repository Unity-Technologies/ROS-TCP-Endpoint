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
from ros_tcp_endpoint.srv import UnityHandshake, UnityHandshakeResponse


class UnityMonitor:
    """
    Class to auto-detect Unity IP.
    """
    def __init__(self, tcp_sender):
        """
        Args:
            tcp_sender:     sends messages to Unity
        """
        self.srv_class = UnityHandshake._request_class()
        self.tcp_sender = tcp_sender
        self.incoming_ip = ''

    def send(self, data):
        message = self.srv_class.deserialize(data)
        if message.ip == '':
            # if the message specifies an IP, Unity has set an IP override, so use it
            self.tcp_sender.process_handshake(self.incoming_ip, message.port)
        else:
            # if not, just talk back to the incoming IP
            self.tcp_sender.process_handshake(message.ip, message.port)
        return UnityHandshakeResponse(self.tcp_sender.unity_ip)
