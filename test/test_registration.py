# Copyright 2026 Unity Technologies
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A client that connects again registers what it had; nothing is destroyed for that."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

from ros_tcp_endpoint.server import SysCommands, TcpServer


class MessageA:
    pass


class MessageB:
    pass


class ServiceA:
    class Request:
        pass


class ServiceB:
    class Request:
        pass


class FakeServer:
    def __init__(self):
        self.publishers_table = {}
        self.subscribers_table = {}
        self.ros_services_table = {}
        self.unity_services_table = {}
        self.executor = Mock()
        self.calls = Mock()
        self.calls.attach_mock(self.executor.remove_node, "remove_node")
        self.log = []
        self.errors = []

    def loginfo(self, text):
        self.log.append(text)

    def send_unity_error(self, text):
        self.errors.append(text)

    def unregister_node(self, old_node):
        # The real method, on this stand-in: the order of its two steps is what is checked.
        TcpServer.unregister_node(self, old_node)


def commands(resolved):
    server = FakeServer()
    sys_commands = SysCommands(server)
    sys_commands.resolve_message_name = lambda name, extension="msg": resolved[name]
    return server, sys_commands


def fake_publisher(topic, message_class, queue_size=10, latch=False):
    return SimpleNamespace(msg=message_class(), queue_size=queue_size, unregister=Mock())


def fake_subscriber(topic, message_class, tcp_server, queue_size=10):
    return SimpleNamespace(msg=message_class, unregister=Mock())


def fake_ros_service(topic, service_class):
    return SimpleNamespace(req=service_class.Request(), unregister=Mock())


def fake_unity_service(topic, service_class, tcp_server, queue_size=10):
    return SimpleNamespace(service_class=service_class, unregister=Mock())


@patch("ros_tcp_endpoint.server.RosPublisher", side_effect=fake_publisher)
def test_publisher_registered_again_is_kept(publisher_class):
    server, sys_commands = commands({"a": MessageA})
    sys_commands.publish("/topic", "a", queue_size=512)
    first = server.publishers_table["/topic"]

    sys_commands.publish("/topic", "a", queue_size=512)

    assert server.publishers_table["/topic"] is first
    assert publisher_class.call_count == 1
    first.unregister.assert_not_called()
    server.executor.remove_node.assert_not_called()
    assert server.executor.add_node.call_count == 1
    assert "already registered" in server.log[-1]


@patch("ros_tcp_endpoint.server.RosPublisher", side_effect=fake_publisher)
def test_publisher_with_another_type_or_queue_size_is_replaced(publisher_class):
    server, sys_commands = commands({"a": MessageA, "b": MessageB})
    sys_commands.publish("/topic", "a", queue_size=10)
    first = server.publishers_table["/topic"]

    sys_commands.publish("/topic", "a", queue_size=512)
    second = server.publishers_table["/topic"]
    assert second is not first
    first.unregister.assert_called_once()

    sys_commands.publish("/topic", "b", queue_size=512)
    assert server.publishers_table["/topic"] is not second
    second.unregister.assert_called_once()
    assert publisher_class.call_count == 3


@patch("ros_tcp_endpoint.server.RosSubscriber", side_effect=fake_subscriber)
def test_subscriber_registered_again_is_kept(subscriber_class):
    server, sys_commands = commands({"a": MessageA, "b": MessageB})
    sys_commands.subscribe("/topic", "a")
    first = server.subscribers_table["/topic"]

    sys_commands.subscribe("/topic", "a")
    assert server.subscribers_table["/topic"] is first
    first.unregister.assert_not_called()
    assert subscriber_class.call_count == 1

    sys_commands.subscribe("/topic", "b")
    assert server.subscribers_table["/topic"] is not first
    first.unregister.assert_called_once()


@patch("ros_tcp_endpoint.server.RosService", side_effect=fake_ros_service)
def test_ros_service_registered_again_is_kept(service_class):
    server, sys_commands = commands({"a": ServiceA, "b": ServiceB})
    sys_commands.ros_service("/service", "a")
    first = server.ros_services_table["/service"]

    sys_commands.ros_service("/service", "a")
    assert server.ros_services_table["/service"] is first
    first.unregister.assert_not_called()
    assert service_class.call_count == 1

    sys_commands.ros_service("/service", "b")
    assert server.ros_services_table["/service"] is not first
    first.unregister.assert_called_once()


@patch("ros_tcp_endpoint.server.UnityService", side_effect=fake_unity_service)
def test_unity_service_registered_again_is_kept(service_class):
    server, sys_commands = commands({"a": ServiceA, "b": ServiceB})
    sys_commands.unity_service("/service", "a")
    first = server.unity_services_table["/service"]

    sys_commands.unity_service("/service", "a")
    assert server.unity_services_table["/service"] is first
    first.unregister.assert_not_called()

    sys_commands.unity_service("/service", "b")
    assert server.unity_services_table["/service"] is not first
    first.unregister.assert_called_once()


@patch("ros_tcp_endpoint.server.RosSubscriber", side_effect=fake_subscriber)
def test_a_replaced_node_leaves_the_executor_before_it_is_destroyed(subscriber_class):
    server, sys_commands = commands({"a": MessageA, "b": MessageB})
    sys_commands.subscribe("/topic", "a")
    first = server.subscribers_table["/topic"]
    server.calls.attach_mock(first.unregister, "unregister")

    sys_commands.subscribe("/topic", "b")

    names = [call[0] for call in server.calls.mock_calls]
    assert names == ["remove_node", "unregister"]
    server.executor.remove_node.assert_called_once_with(first)
