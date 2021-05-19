from unittest import mock
from unittest.mock import Mock
from ros_tcp_endpoint.client import ClientThread
from ros_tcp_endpoint.server import TcpServer


@mock.patch("threading.Thread")
def test_client_constructor(mock_threading):
    tcp_server = TcpServer(
        node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000
    )
    # client_thread = ClientThread(Mock(), tcp_server, Mock(), Mock())
