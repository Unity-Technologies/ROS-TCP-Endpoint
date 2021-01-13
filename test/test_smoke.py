import pytest
from unittest import mock
import ros_tcp_endpoint

@mock.patch('ros_tcp_endpoint.publisher.rospy')
def test_pub_constructor(mock_pub_rospy):
    mock_msg_pub = mock.Mock()
    pub = ros_tcp_endpoint.RosPublisher("/fake_topic", mock_msg_pub)
    # Message is constructed in publisher, stored as object instance
    mock_msg_pub.assert_called_once()

@mock.patch('ros_tcp_endpoint.subscriber.rospy')
def test_sub_constructor(mock_sub_rospy):
    mock_msg_sub = mock.Mock()
    sub = ros_tcp_endpoint.RosSubscriber("/fake_topic2", mock_msg_sub, None)
    # Message is NOT constructed in subscriber, stored as class reference
    mock_msg_sub.assert_not_called()

@mock.patch('ros_tcp_endpoint.service.rospy')
def test_srv_constructor(mock_srv_rospy):
    mock_srv = mock.Mock()
    src = ros_tcp_endpoint.RosService("fake_service", mock_srv)
    mock_srv._request_class.assert_called_once()
