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

from .communication import RosReceiver


class RosSubscriber(RosReceiver):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, tcp_server, queue_size=10):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        RosReceiver.__init__(self)
        self.topic = topic
        self.node_name = "{}_subscriber".format(topic)
        self.tcp_server = tcp_server
        self.queue_size = queue_size

        # Start Subscriber listener function
        self.sub = rospy.Subscriber(self.topic, message_class, self.send)

    def will_block_for_response(self):
        return False

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network
        """

        self.tcp_server.send_unity_message(self.topic, data)

    def unregister(self):
        """

        Returns:

        """
        if not self.sub is None:
            self.sub.unregister()
