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
Bridge a client-implemented Action server through ROS-TCP-Endpoint.

When a ROS 2 action client sends a goal, the execute_callback
forwards it to the client using the same __request/__response two-frame
pair that regular unity_service uses. The client computes and sends the
result back; feedback is pushed from the client via a separate
__action_publish_feedback syscommand.
"""

import re
import threading
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.serialization import deserialize_message, serialize_message

from .communication import RosSender


class RosActionServer(RosSender):

    def __init__(self, action_name, action_class, tcp_server):
        stripped = re.sub("[^A-Za-z0-9_]+", "", action_name)
        node_name = f"{stripped}_RosActionServer"
        RosSender.__init__(self, node_name)

        self.action_name = action_name
        self.action_class = action_class
        self.tcp_server = tcp_server

        # goal_uuid bytes -> ServerGoalHandle (for feedback publishing)
        self._goal_handles = {}
        self._lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            action_class,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def _execute_callback(self, goal_handle):
        """Forward the goal to the client and wait for the result."""
        goal_uuid = bytes(goal_handle.goal_id.uuid)

        with self._lock:
            self._goal_handles[goal_uuid] = goal_handle

        # Serialize the Goal body.
        goal_msg = goal_handle.request
        goal_cdr = serialize_message(goal_msg)

        # Send the goal to the client using the __request / __response
        # mechanism. send_unity_service_request blocks until the client
        # responds. We use the action_name as the topic/destination
        # so the client.s action server can route it.
        #
        # BUT: send_unity_service_request calls deserialize_message
        # on the response, which needs a rclpy type. We need raw CDR
        # bytes instead. So we use a raw version.
        result_cdr = self._send_goal_to_client(goal_uuid, goal_cdr)

        # Clean up.
        with self._lock:
            self._goal_handles.pop(goal_uuid, None)

        if result_cdr is None:
            goal_handle.abort()
            return self.action_class.Result()

        result_msg = deserialize_message(result_cdr, self.action_class.Result)
        goal_handle.succeed()
        return result_msg

    def _send_goal_to_client(self, goal_uuid, goal_cdr):
        """Send a goal to the client and wait for the result CDR bytes.

        Uses the same ThreadPauser mechanism as send_unity_service_request
        but works with raw CDR bytes.
        """
        from .tcp_sender import ThreadPauser, SysCommand_Service

        sender = self.tcp_server.unity_tcp_sender
        if sender.queue is None:
            return None

        thread_pauser = ThreadPauser()
        with sender.srv_lock:
            srv_id = sender.next_srv_id
            sender.next_srv_id += 1
            sender.services_waiting[srv_id] = thread_pauser

        # Build __request{srv_id} header.
        from .client import ClientThread
        command = SysCommand_Service()
        command.srv_id = srv_id
        serialized_header = ClientThread.serialize_command("__request", command)

        # Build the data frame: destination = action_name, payload =
        # 16-byte goal UUID + CDR goal body. the client strips the UUID
        # to identify which goal this is for.
        import struct
        dest_bytes = self.action_name.encode("utf-8")
        dest_len = len(dest_bytes)
        payload = goal_uuid + goal_cdr
        dest_info = struct.pack("<I%ss" % dest_len, dest_len, dest_bytes)
        msg_length = struct.pack("<I", len(payload))
        serialized_message = dest_info + msg_length + payload

        sender.queue.put(b"".join([serialized_header, serialized_message]))

        # Block until the client sends __response{srv_id} with the result.
        thread_pauser.sleep_until_resumed()

        return thread_pauser.result  # raw CDR bytes

    def publish_feedback(self, goal_uuid_bytes, feedback_cdr):
        """Called when the client sends feedback for an in-progress goal."""
        with self._lock:
            goal_handle = self._goal_handles.get(goal_uuid_bytes)

        if goal_handle is None:
            self.get_logger().warning(
                f"publish_feedback: no goal handle for {goal_uuid_bytes.hex()}")
            return

        fb_msg = deserialize_message(feedback_cdr, self.action_class.Feedback)
        goal_handle.publish_feedback(fb_msg)

    def unregister(self):
        self._action_server.destroy()
        self.destroy_node()
