#!/usr/bin/env python

import rospy
import actionlib_msgs.msg
import diagnostic_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
import stereo_msgs.msg
import trajectory_msgs.msg
import visualization_msgs.msg

from ros_tcp_endpoint import TcpServer


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")
    tcp_server = TcpServer(ros_node_name)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
