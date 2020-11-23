#!/usr/bin/env python

import rospy
from tcp_endpoint.srv import RosUnityHandshake, RosUnityHandshakeResponse

class RosUnityHandshakeService:
    """
    Class to auto-detect Unity IP.
    """
    def __init__(self, tcp_sender):
        """
        Args:
            tcp_sender:     sends messages to Unity
        """
        self.srv_class = RosUnityHandshake._request_class()
        self.tcp_sender = tcp_sender
        self.incoming_ip = ''

    # The client thread lets us know what the incoming IP is, so we can use it later
    def set_thread(self, tcp_client_thread):
        self.incoming_ip = tcp_client_thread.incoming_ip

    def send(self, data):
        message = self.srv_class.deserialize(data)
        if message.ip == '': # if the message specifies an IP, Unity has set an IP override, so use it
            self.tcp_sender.process_handshake(self.incoming_ip, message.port)
        else: # if not, just talk back to the incoming IP
            self.tcp_sender.process_handshake(message.ip, message.port)
        return RosUnityHandshakeResponse(self.tcp_sender.unity_ip)
