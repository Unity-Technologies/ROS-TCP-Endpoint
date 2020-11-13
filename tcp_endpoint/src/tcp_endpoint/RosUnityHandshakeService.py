#!/usr/bin/env python

import rospy
from tcp_endpoint.srv import RosUnityHandshake, RosUnityHandshakeResponse

class RosUnityHandshakeService:
    """
    Class to handle system messages.
    """
    def __init__(self, tcp_sender):
        """
        Args:
            service:        The service name in ROS
            service_class:  The service class in catkin workspace
        """
        self.srv_class = RosUnityHandshake._request_class()
        self.tcp_sender = tcp_sender
        self.incoming_ip = ''

    def set_thread(self, tcp_client_thread):
        self.incoming_ip = tcp_client_thread.incoming_ip

    def send(self, data):
        message = self.srv_class.deserialize(data)
        if message.ip == '':
            self.tcp_sender.process_handshake(self.incoming_ip, message.port)
        else:
            self.tcp_sender.process_handshake(message.ip, message.port)
        return RosUnityHandshakeResponse(self.tcp_sender.unity_ip)
