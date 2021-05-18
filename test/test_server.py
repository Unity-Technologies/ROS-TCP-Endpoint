from unittest import mock
from ros_tcp_endpoint import TcpServer


@mock.patch("socket.socket")
@mock.patch("ros_tcp_endpoint.server.rospy")
def test_server_constructor(mock_ros, mock_socket):
    mock_ros.get_param = mock.Mock(return_value="127.0.0.1")
    server = TcpServer("test-tcp-server")
    assert server.node_name == "test-tcp-server"
    assert server.tcp_ip == "127.0.0.1"
    assert server.buffer_size == 1024
    assert server.connections == 2


def test_start_server():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    assert server.tcp_ip == "127.0.0.1"
    assert server.tcp_port == 10000
    assert server.connections == 2
    server.start()



