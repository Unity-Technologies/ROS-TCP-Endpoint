import genpy
import rospy
import re

import traceback
from rospy import ROSSerializationException, ROSException, TransportTerminated

from .communication import RosSender


def publish_no_serialize(self, raw_data):
    if self.closed:
        # during shutdown, the topic can get closed, which creates
        # a race condition with user code testing is_shutdown
        if not rospy.topics.is_shutdown():
            raise ROSException("publish() to a closed topic")
        else:
            return

    if self.is_latch:
        self.latch = raw_data

    if not self.has_connections():
        # publish() falls through
        return False

    conns = self.connections

    # #2128 test our buffer. I don't know how this got closed in
    # that case, but we can at least diagnose the problem.
    b = self.buff

    try:
        b.tell()

        # IMPORTANT: TCP-Connector needs to increment the seq value now, or we need to find the byte in our buffer
        #            and manually add one
        # self.seq += 1  # count messages published to the topic

        b.write(raw_data)

        # send the buffer to all connections
        err_con = []
        data = b.getvalue()

        for c in conns:
            try:
                if not rospy.topics.is_shutdown():
                    c.write_data(data)
            except TransportTerminated as e:
                rospy.core.logdebug(
                    "publisher connection to [%s] terminated, see errorlog for details:\n%s"
                    % (c.endpoint_id, traceback.format_exc())
                )
                err_con.append(c)
            except Exception as e:
                # greater severity level
                rospy.core.logdebug(
                    "publisher connection to [%s] terminated, see errorlog for details:\n%s"
                    % (c.endpoint_id, traceback.format_exc())
                )
                err_con.append(c)

        # reset the buffer and update stats
        self.message_data_sent += b.tell()  # STATS
        b.seek(0)
        b.truncate(0)

    except ValueError:
        # operations on self.buff can fail if topic is closed
        # during publish, which often happens during Ctrl-C.
        # diagnose the error and report accordingly.
        if self.closed:
            if rospy.topics.is_shutdown():
                # we offer no guarantees on publishes that occur
                # during shutdown, so this is not exceptional.
                return
            else:
                # this indicates that user-level code most likely
                # closed the topic, which is exceptional.
                raise ROSException("topic was closed during publish()")
        else:
            # unexpected, so re-raise original error
            raise

    # remove any bad connections
    for c in err_con:
        try:
            # connection will callback into remove_connection when
            # we close it
            c.close()
        except:
            pass


class RosPublisher(RosSender):
    """
    Class to publish messages to a ROS topic
    """

    # TODO: surface latch functionality
    def __init__(self, topic, message_class, queue_size=10, latch=False):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        strippedTopic = re.sub("[^A-Za-z0-9_]+", "", topic)
        node_name = "{}_RosPublisher".format(strippedTopic)
        RosSender.__init__(self, node_name)
        self.msg = message_class()
        self.pub = rospy.Publisher(topic, message_class, queue_size=queue_size, latch=latch)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's message class, and publishes the message to ROS topic.

        Args:
            data: The already serialized message_class data coming from outside of ROS

        Returns:
            None: Explicitly return None so behaviour can be
        """
        try:
            self.pub.impl.acquire()
            publish_no_serialize(self.pub.impl, data)
        except genpy.SerializationError as e:
            # can't go to rospy.logerr(), b/c this could potentially recurse
            rospy.topics._logger.error(traceback.format_exc())
            raise ROSSerializationException(str(e))
        finally:
            self.pub.impl.release()

    def unregister(self):
        """

        Returns:

        """
        self.pub.unregister()
