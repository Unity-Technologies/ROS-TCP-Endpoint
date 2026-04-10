#  Copyright 2020 Unity Technologies
#  Copyright 2026 gd-ros-tcp-connector contributors
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

"""
Thin wrapper around ``rclpy.action.ActionClient`` that bridges the
ROS-TCP-Endpoint wire protocol to real ROS 2 Action servers.

A plain ``rclpy.create_client()`` cannot discover Action endpoints
because they use a different DDS endpoint kind / QoS profile than
regular services.  This module solves the problem by creating a real
``ActionClient`` and exposing ``send_goal`` / ``get_result`` /
``cancel_goal`` as methods the TCP server can call with serialized
CDR payloads.
"""

import re
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.serialization import deserialize_message, serialize_message

from .communication import RosSender


class RosActionClient(RosSender):
    """
    Manages one ``rclpy.action.ActionClient`` for a single action name.
    """

    def __init__(self, action_name, action_class, tcp_server):
        stripped = re.sub("[^A-Za-z0-9_]+", "", action_name)
        node_name = f"{stripped}_RosActionClient"
        RosSender.__init__(self, node_name)

        self.action_name = action_name
        self.action_class = action_class
        self.tcp_server = tcp_server

        self._action_client = ActionClient(self, action_class, action_name)
        self._goal_handles = {}  # goal_uuid_bytes -> ClientGoalHandle
        self._goal_handles_lock = threading.Lock()

    def send_goal(self, goal_data):
        """Send a goal. *goal_data* is CDR-serialized Goal body bytes.

        Returns the CDR-serialized SendGoal_Response (accepted + stamp),
        or None on failure.
        """
        import time

        goal_msg = deserialize_message(
            goal_data, self.action_class.Goal)

        # The node is already added to the TcpServer's
        # MultiThreadedExecutor, so DDS discovery proceeds on the
        # executor thread. We just poll server_is_ready() here; do NOT
        # call rclpy.spin_once(self) because the executor already owns
        # this node and double-spinning causes deadlocks.
        server_ready = False
        for _ in range(100):  # 100 x 0.1s = 10s max
            if self._action_client.server_is_ready():
                server_ready = True
                break
            time.sleep(0.1)

        if not server_ready:
            self.get_logger().error(
                f"Action server {self.action_name} not available "
                f"(waited 10s — is the server running?)")
            return None

        # Do NOT pass feedback_callback here. The client subscribes to the
        # feedback topic via __subscribe, which creates a RosSubscriber
        # that forwards feedback through the normal topic path. If we
        # ALSO forwarded feedback from _on_feedback, every feedback
        # would arrive twice on the client side.
        future = self._action_client.send_goal_async(goal_msg)

        # Wait for the send_goal future. Again, the executor is
        # spinning on another thread, so we just poll the future.
        for _ in range(100):  # 10s max
            if future.done():
                break
            time.sleep(0.1)
        goal_handle = future.result()
        if goal_handle is None:
            return None

        # Build a SendGoal_Response to send back to the client.
        # rclpy's ActionClient generates its own UUID for the goal
        # (ignoring any UUID we might have set), so we need to tell
        # the client which UUID was actually used. We prepend the 16-byte
        # UUID to the CDR-serialized SendGoal_Response; the client strips
        # the first 16 bytes before deserializing.
        response_class = self.action_class.Impl.SendGoalService.Response
        resp = response_class()
        resp.accepted = goal_handle.accepted
        resp.stamp = goal_handle.stamp if hasattr(goal_handle, 'stamp') else resp.stamp

        goal_uuid = bytes(goal_handle.goal_id.uuid)
        if goal_handle.accepted:
            with self._goal_handles_lock:
                self._goal_handles[goal_uuid] = goal_handle

        return goal_uuid + serialize_message(resp)

    def get_result(self, goal_id_data):
        """Request the result for a goal. *goal_id_data* is CDR-serialized
        GetResult_Request bytes (contains the goal UUID).

        Returns CDR-serialized GetResult_Response, or None on failure.
        """
        request_class = self.action_class.Impl.GetResultService.Request
        req = deserialize_message(goal_id_data, request_class)

        key = bytes(req.goal_id.uuid)
        with self._goal_handles_lock:
            goal_handle = self._goal_handles.get(key)

        if goal_handle is None:
            self.get_logger().error(
                f"get_result: no goal handle for UUID {key.hex()}")
            return None

        import time
        result_future = goal_handle.get_result_async()
        # Poll instead of spin_until_future_complete — executor owns us.
        for _ in range(3000):  # 300s max
            if result_future.done():
                break
            time.sleep(0.1)
        result = result_future.result()

        # Clean up the handle.
        with self._goal_handles_lock:
            self._goal_handles.pop(key, None)

        if result is None:
            return None

        # Build GetResult_Response
        response_class = self.action_class.Impl.GetResultService.Response
        resp = response_class()
        resp.status = result.status
        resp.result = result.result
        return serialize_message(resp)

    def cancel_goal(self, goal_info_data):
        """Cancel a goal. *goal_info_data* is CDR-serialized
        CancelGoal_Request bytes.

        Returns CDR-serialized CancelGoal_Response, or None.
        """
        from action_msgs.srv import CancelGoal
        req = deserialize_message(goal_info_data, CancelGoal.Request)

        key = bytes(req.goal_info.goal_id.uuid)
        with self._goal_handles_lock:
            goal_handle = self._goal_handles.get(key)

        if goal_handle is None:
            self.get_logger().error(
                f"cancel_goal: no goal handle for UUID {key.hex()}")
            return None

        import time
        cancel_future = goal_handle.cancel_goal_async()
        for _ in range(100):  # 10s max
            if cancel_future.done():
                break
            time.sleep(0.1)
        cancel_response = cancel_future.result()

        if cancel_response is None:
            return None
        return serialize_message(cancel_response)

    def unregister(self):
        self._action_client.destroy()
        self.destroy_node()
