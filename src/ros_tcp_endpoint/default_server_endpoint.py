#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer


def main(args=None):
    tcp_server = TcpServer("UnityEndpoint")

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
