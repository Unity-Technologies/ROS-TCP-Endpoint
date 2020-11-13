#!/usr/bin/env python
import StringIO

import rospy
import socket

from tcp_endpoint.RosCommunication import RosReceiver
from tcp_endpoint.RosTCPClientThread import ClientThread


class RosSubscriber(RosReceiver):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, tcp_sender, queue_size=10):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        self.topic = topic
        self.node_name = "{}_subsciber".format(topic)
        self.msg = message_class
        self.tcp_sender = tcp_sender
        self.queue_size = queue_size

        # Start Subscriber listener function
        self.listener()

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network

        Returns:
            self.msg: The deserialize message

        """

        self.tcp_sender.send_unity_message(self.topic, data)
        return self.msg

    def listener(self):
        """

        Returns:

        """
        rospy.Subscriber(self.topic, self.msg, self.send)
