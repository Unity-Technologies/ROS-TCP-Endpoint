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

import rclpy
import socket
import json
import sys
import threading
import importlib

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import deserialize_message

from .tcp_sender import UnityTcpSender
from .client import ClientThread
from .subscriber import RosSubscriber
from .publisher import RosPublisher
from .service import RosService
from .unity_service import UnityService
from .action_client import RosActionClient
from .action_server import RosActionServer


class TcpServer(Node):
    """
    Initializes ROS node and TCP server.
    """

    def __init__(self, node_name, buffer_size=1024, connections=10, tcp_ip=None, tcp_port=None):
        """
        Initializes ROS node and class variables.

        Args:
            node_name:               ROS node name for executing code
            buffer_size:             The read buffer size used when reading from a socket
            connections:             Max number of queued connections. See Python Socket documentation
        """
        super().__init__(node_name)

        self.declare_parameter("ROS_IP", "0.0.0.0")
        self.declare_parameter("ROS_TCP_PORT", 10000)

        if tcp_ip:
            self.loginfo("Using ROS_IP override from constructor: {}".format(tcp_ip))
            self.tcp_ip = tcp_ip
        else:
            self.tcp_ip = self.get_parameter("ROS_IP").get_parameter_value().string_value

        if tcp_port:
            self.loginfo("Using ROS_TCP_PORT override from constructor: {}".format(tcp_port))
            self.tcp_port = tcp_port
        else:
            self.tcp_port = self.get_parameter("ROS_TCP_PORT").get_parameter_value().integer_value

        self.unity_tcp_sender = UnityTcpSender(self)

        self.node_name = node_name
        self.publishers_table = {}
        self.subscribers_table = {}
        self.ros_services_table = {}
        self.unity_services_table = {}
        self.action_clients_table = {}
        self.action_servers_table = {}
        self.pending_action_op = None
        self.pending_action_name = None
        self.pending_action_goal_uuid = None
        self.buffer_size = buffer_size
        self.connections = connections
        self.syscommands = SysCommands(self)
        self.pending_srv_id = None
        self.pending_srv_is_request = False

    def start(self, publishers=None, subscribers=None):
        if publishers is not None:
            self.publishers_table = publishers
        if subscribers is not None:
            self.subscribers_table = subscribers
        server_thread = threading.Thread(target=self.listen_loop)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()

    def listen_loop(self):
        """
            Creates and binds sockets using TCP variables then listens for incoming connections.
            For each new connection a client thread will be created to handle communication.
        """
        self.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.tcp_ip, self.tcp_port))

        while True:
            tcp_server.listen(self.connections)

            try:
                (conn, (ip, port)) = tcp_server.accept()
                ClientThread(conn, self, ip, port).start()
            except socket.timeout as err:
                self.logerr("ros_tcp_endpoint.TcpServer: socket timeout")

    def send_unity_error(self, error):
        self.unity_tcp_sender.send_unity_error(error)

    def send_unity_message(self, topic, message):
        self.unity_tcp_sender.send_unity_message(topic, message)

    def send_unity_service(self, topic, service_class, request):
        return self.unity_tcp_sender.send_unity_service_request(topic, service_class, request)

    def send_unity_service_response(self, srv_id, data):
        self.unity_tcp_sender.send_unity_service_response(srv_id, data)

    def handle_syscommand(self, topic, data):
        function = getattr(self.syscommands, topic[2:])
        if function is None:
            self.send_unity_error("Don't understand SysCommand.'{}'".format(topic))
        else:
            message_json = data.decode("utf-8")[:-1]
            params = json.loads(message_json)
            function(**params)

    def loginfo(self, text):
        self.get_logger().info(text)

    def logwarn(self, text):
        self.get_logger().warning(text)

    def logerr(self, text):
        self.get_logger().error(text)

    def setup_executor(self):
        """
            Since rclpy.spin() is a blocking call the server needed a way
            to spin all of the relevant nodes at the same time.

            MultiThreadedExecutor allows us to set the number of threads
            needed as well as the nodes that need to be spun.
        """
        num_threads = (
            len(self.publishers_table.keys())
            + len(self.subscribers_table.keys())
            + len(self.ros_services_table.keys())
            + len(self.unity_services_table.keys())
            + 1
        )
        executor = MultiThreadedExecutor(num_threads)

        executor.add_node(self)

        for ros_node in self.publishers_table.values():
            executor.add_node(ros_node)
        for ros_node in self.subscribers_table.values():
            executor.add_node(ros_node)
        for ros_node in self.ros_services_table.values():
            executor.add_node(ros_node)
        for ros_node in self.unity_services_table.values():
            executor.add_node(ros_node)

        self.executor = executor
        executor.spin()

    def unregister_node(self, old_node):
        if old_node is not None:
            old_node.unregister()
            if self.executor is not None:
                self.executor.remove_node(old_node)

    def destroy_nodes(self):
        """
            Clean up all of the nodes
        """
        for ros_node in self.publishers_table.values():
            ros_node.destroy_node()
        for ros_node in self.subscribers_table.values():
            ros_node.destroy_node()
        for ros_node in self.ros_services_table.values():
            ros_node.destroy_node()
        for ros_node in self.unity_services_table.values():
            ros_node.destroy_node()

        self.destroy_node()


class SysCommands:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server

    def subscribe(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't subscribe to a blank topic name! SysCommand.subscribe({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.subscribe - Unknown message class '{}'".format(message_name)
            )
            return

        old_node = self.tcp_server.subscribers_table.get(topic)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_subscriber = RosSubscriber(topic, message_class, self.tcp_server)
        self.tcp_server.subscribers_table[topic] = new_subscriber
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_subscriber)

        self.tcp_server.loginfo("RegisterSubscriber({}, {}) OK".format(topic, message_class))

    def publish(self, topic, message_name, queue_size=10, latch=False):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't publish to a blank topic name! SysCommand.publish({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.publish - Unknown message class '{}'".format(message_name)
            )
            return

        old_node = self.tcp_server.publishers_table.get(topic)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_publisher = RosPublisher(topic, message_class, queue_size=queue_size, latch=latch)

        self.tcp_server.publishers_table[topic] = new_publisher
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_publisher)

        self.tcp_server.loginfo("RegisterPublisher({}, {}) OK".format(topic, message_class))

    def ros_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return
        message_class = self.resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        old_node = self.tcp_server.ros_services_table.get(topic)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_service = RosService(topic, message_class)

        self.tcp_server.ros_services_table[topic] = new_service
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_service)

        self.tcp_server.loginfo("RegisterRosService({}, {}) OK".format(topic, message_class))

    def unity_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = self.resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        old_node = self.tcp_server.unity_services_table.get(topic)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_service = UnityService(str(topic), message_class, self.tcp_server)

        self.tcp_server.unity_services_table[topic] = new_service
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_service)

        self.tcp_server.loginfo("RegisterUnityService({}, {}) OK".format(topic, message_class))

    def action_client(self, action_name, action_type):
        """Register a RosActionClient that bridges to a real ROS2 action
        server.  Unlike ``ros_service``, this creates an
        ``rclpy.action.ActionClient`` which can discover DDS action
        endpoints that plain ``create_client()`` cannot see.

        The client sends ``__action_client {action_name, action_type}``
        once, then uses ``__action_send_goal``, ``__action_get_result``,
        and ``__action_cancel_goal`` to interact with the server.
        """
        if action_name == "":
            self.tcp_server.send_unity_error(
                "RegisterActionClient - blank action name!")
            return

        # Resolve the Action class (e.g. example_interfaces.action.Fibonacci).
        action_class = self.resolve_message_name(action_type, "action")
        if action_class is None:
            self.tcp_server.send_unity_error(
                "RegisterActionClient({}, {}) - Unknown action class '{}'".format(
                    action_name, action_type, action_type))
            return

        old_node = self.tcp_server.action_clients_table.get(action_name)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_client = RosActionClient(action_name, action_class,
                                     self.tcp_server.unity_tcp_sender)
        self.tcp_server.action_clients_table[action_name] = new_client
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_client)

        self.tcp_server.loginfo(
            "RegisterActionClient({}, {}) OK".format(action_name, action_class))

    def action_send_goal(self, action_name, srv_id):
        """The next frame carries the CDR-serialized Goal body.  We set
        pending_srv_id so the client thread routes the next payload to
        our action_client.send_goal, and sends the response back via
        the normal __response{srv_id} mechanism.
        """
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True
        self.tcp_server.pending_action_name = action_name
        self.tcp_server.pending_action_op = "send_goal"

    def action_get_result(self, action_name, srv_id):
        """The next frame carries GetResult_Request (just a UUID)."""
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True
        self.tcp_server.pending_action_name = action_name
        self.tcp_server.pending_action_op = "get_result"

    def action_cancel_goal(self, action_name, srv_id):
        """The next frame carries CancelGoal_Request."""
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True
        self.tcp_server.pending_action_name = action_name
        self.tcp_server.pending_action_op = "cancel_goal"

    def action_server(self, action_name, action_type):
        """Register a RosActionServer so the client can implement an action.

        When a ROS 2 action client sends a goal, the endpoint forwards
        it to the client as a __request/__response pair. The client processes the
        goal and sends feedback via __action_publish_feedback and the
        result via __response.
        """
        if action_name == "":
            self.tcp_server.send_unity_error(
                "RegisterActionServer - blank action name!")
            return

        action_class = self.resolve_message_name(action_type, "action")
        if action_class is None:
            self.tcp_server.send_unity_error(
                "RegisterActionServer({}, {}) - Unknown action class".format(
                    action_name, action_type))
            return

        old_node = self.tcp_server.action_servers_table.get(action_name)
        if old_node is not None:
            self.tcp_server.unregister_node(old_node)

        new_server = RosActionServer(action_name, action_class,
                                     self.tcp_server)
        self.tcp_server.action_servers_table[action_name] = new_server
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(new_server)

        self.tcp_server.loginfo(
            "RegisterActionServer({}, {}) OK".format(action_name, action_class))

    def action_publish_feedback(self, action_name, goal_uuid_hex):
        """The next frame carries CDR-serialized Feedback body.

        We set pending state so the client thread routes the next
        payload to the matching RosActionServer.publish_feedback().
        """
        self.tcp_server.pending_srv_id = None  # not a srv_id response
        self.tcp_server.pending_action_name = action_name
        self.tcp_server.pending_action_op = "publish_feedback"
        self.tcp_server.pending_action_goal_uuid = goal_uuid_hex

    def response(self, srv_id):  # the next message is a service response
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = False

    def request(self, srv_id):  # the next message is a service request
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True

    def topic_list(self):
        self.tcp_server.unity_tcp_sender.send_topic_list()

    def resolve_message_name(self, name, extension="msg"):
        """Resolve a ROS message/service/action class by name.

        Tries the given extension first (e.g. "msg" or "srv"), then
        falls back to "action" if the primary lookup fails. This lets
        clients register Action-generated types (e.g.
        ``Fibonacci_SendGoal``) via the normal ``__ros_service`` /
        ``__subscribe`` syscommands without any protocol changes.

        Also handles 3-segment names like
        ``package/action/ClassName`` (which __topic_list may report)
        by extracting the middle segment as the extension.
        """
        # Handle 3-segment names: "pkg/msg/Type" or "pkg/action/Type"
        parts = name.split("/")
        if len(parts) == 3 and parts[1] in ("msg", "srv", "action"):
            name = parts[0] + "/" + parts[2]
            extension = parts[1]

        result = self._try_resolve_message_name(name, extension)
        if result is None and extension != "action":
            result = self._try_resolve_message_name(name, "action")
        if result is None:
            self.tcp_server.logerr(
                "Failed to resolve message name '{}' in extensions '{}' and 'action'".format(
                    name, extension
                )
            )
        return result

    def _try_resolve_message_name(self, name, extension):
        """Attempt to import *name* from the *extension* sub-module.

        Returns the class on success, or ``None`` on any failure
        (without logging — the caller decides whether to report).

        For the ``"action"`` extension, rclpy exposes Action sub-types
        as nested classes rather than flat module attributes.  For
        example, ``example_interfaces/Fibonacci_SendGoal_Request`` maps
        to ``example_interfaces.action.Fibonacci.Impl.SendGoalService.Request``.
        This method handles the translation automatically so that
        clients can use the flat ``Package/Action_Suffix`` naming
        convention over the wire.
        """
        try:
            names = name.split("/")
            module_name = names[0]
            class_name = names[1]
            importlib.import_module(module_name + "." + extension)
            module = sys.modules[module_name]
            if module is None:
                return None
            module = getattr(module, extension, None)
            if module is None:
                return None

            # For msg/srv the class sits directly on the sub-module.
            cls = getattr(module, class_name, None)
            if cls is not None:
                return cls

            # For action types, try the nested-class lookup.
            if extension == "action":
                cls = self._try_resolve_action_class(module, class_name)
            return cls
        except (IndexError, KeyError, AttributeError, ImportError):
            return None

    @staticmethod
    def _try_resolve_action_class(action_module, class_name):
        """Resolve an Action sub-type from its flat wire name.

        rclpy generates Action classes with this nesting structure::

            <ActionModule>.<Action>.Goal
            <ActionModule>.<Action>.Result
            <ActionModule>.<Action>.Feedback
            <ActionModule>.<Action>.Impl.SendGoalService.Request
            <ActionModule>.<Action>.Impl.SendGoalService.Response
            <ActionModule>.<Action>.Impl.GetResultService.Request
            <ActionModule>.<Action>.Impl.GetResultService.Response
            <ActionModule>.<Action>.Impl.FeedbackMessage

        On the wire the client sends a flat name like
        ``Fibonacci_SendGoal_Request``.  We split on ``_`` to recover
        the Action name and the suffix, then walk the nested attrs.
        """
        try:
            # Map flat suffixes to attribute paths inside Action.Impl.
            # Longest suffixes first so "SendGoal_Request" is tried
            # before "SendGoal".
            IMPL_MAP = {
                "SendGoal_Request":  ["Impl", "SendGoalService", "Request"],
                "SendGoal_Response": ["Impl", "SendGoalService", "Response"],
                "GetResult_Request":  ["Impl", "GetResultService", "Request"],
                "GetResult_Response": ["Impl", "GetResultService", "Response"],
                "FeedbackMessage":   ["Impl", "FeedbackMessage"],
                # Service-class lookups (no _Request/_Response suffix).
                # ros_service resolves the service CLASS, then accesses
                # .Request / .Response internally.  The service class
                # itself lives at Action.Impl.{SendGoal,GetResult}Service.
                "SendGoal":  ["Impl", "SendGoalService"],
                "GetResult": ["Impl", "GetResultService"],
            }
            # Direct sub-class suffixes (no Impl nesting).
            DIRECT_MAP = {
                "Goal": "Goal",
                "Result": "Result",
                "Feedback": "Feedback",
            }

            # Try each known suffix, longest first so that
            # "SendGoal_Request" matches before "Goal".
            for suffix, path in IMPL_MAP.items():
                if class_name.endswith("_" + suffix):
                    action_name = class_name[: -(len(suffix) + 1)]
                    obj = getattr(action_module, action_name, None)
                    for attr in path:
                        if obj is None:
                            break
                        obj = getattr(obj, attr, None)
                    return obj

            for suffix, attr in DIRECT_MAP.items():
                if class_name.endswith("_" + suffix):
                    action_name = class_name[: -(len(suffix) + 1)]
                    obj = getattr(action_module, action_name, None)
                    if obj is not None:
                        return getattr(obj, attr, None)

            return None
        except (AttributeError, TypeError):
            return None
