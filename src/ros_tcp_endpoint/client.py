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

import struct
import socket
import rospy
from io import BytesIO

import threading
import json

from .exceptions import TopicOrServiceNameDoesNotExistError


class ClientThread(threading.Thread):
    """
    Thread class to read all data from a connection and pass along the data to the
    desired source.
    """

    def __init__(self, conn, tcp_server, incoming_ip, incoming_port):
        """
        Set class variables
        Args:
            conn:
            source_destination_dict: dictionary of destination name to RosCommunicator class
        """
        self.conn = conn
        self.tcp_server = tcp_server
        self.incoming_ip = incoming_ip
        self.incoming_port = incoming_port
        threading.Thread.__init__(self)

    @staticmethod
    def recvall(conn, size, flags=0):
        """
        Receive exactly bufsize bytes from the socket.
        """
        buffer = bytearray(size)
        view = memoryview(buffer)
        pos = 0
        while pos < size:
            read = conn.recv_into(view[pos:], size - pos, flags)
            if not read:
                raise IOError("No more data available")
            pos += read
        return bytes(buffer)

    @staticmethod
    def read_int32(conn):
        """
        Reads four bytes from socket connection and unpacks them to an int

        Returns: int

        """
        raw_bytes = ClientThread.recvall(conn, 4)
        num = struct.unpack("<I", raw_bytes)[0]
        return num

    @staticmethod
    def read_string(conn):
        """
        Reads int32 from socket connection to determine how many bytes to
        read to get the string that follows. Read that number of bytes and
        decode to utf-8 string.

        Returns: string

        """
        str_len = ClientThread.read_int32(conn)

        str_bytes = ClientThread.recvall(conn, str_len)
        decoded_str = str_bytes.decode("utf-8")

        return decoded_str

    @staticmethod
    def read_message(conn):
        """
        Decode destination and full message size from socket connection.
        Grab bytes in chunks until full message has been read.
        """
        data = b""

        destination = ClientThread.read_string(conn)
        full_message_size = ClientThread.read_int32(conn)

        data = ClientThread.recvall(conn, full_message_size)

        if full_message_size > 0 and not data:
            rospy.logerr("No data for a message size of {}, breaking!".format(full_message_size))
            return

        return destination, data

    @staticmethod
    def serialize_message(destination, message):
        """
        Serialize a destination and message class.

        Args:
            destination: name of destination
            message:     message class to serialize

        Returns:
            serialized destination and message as a list of bytes
        """
        dest_bytes = destination.encode("utf-8")
        length = len(dest_bytes)
        dest_info = struct.pack("<I%ss" % length, length, dest_bytes)

        serial_response = BytesIO()
        message.serialize(serial_response)

        # Per documention, https://docs.python.org/3.8/library/io.html#io.IOBase.seek,
        # seek to end of stream for length
        # SEEK_SET or 0 - start of the stream (the default); offset should be zero or positive
        # SEEK_CUR or 1 - current stream position; offset may be negative
        # SEEK_END or 2 - end of the stream; offset is usually negative
        response_len = serial_response.seek(0, 2)

        msg_length = struct.pack("<I", response_len)
        serialized_message = dest_info + msg_length + serial_response.getvalue()

        return serialized_message

    @staticmethod
    def serialize_command(command, params):
        cmd_bytes = command.encode("utf-8")
        cmd_length = len(cmd_bytes)
        cmd_info = struct.pack("<I%ss" % cmd_length, cmd_length, cmd_bytes)

        json_bytes = json.dumps(params.__dict__).encode("utf-8")
        json_length = len(json_bytes)
        json_info = struct.pack("<I%ss" % json_length, json_length, json_bytes)

        return cmd_info + json_info

    def send_ros_service_request(self, srv_id, destination, data):
        if destination not in self.tcp_server.source_destination_dict.keys():
            error_msg = "Service destination '{}' is not registered! Known topics are: {} ".format(
                destination, self.tcp_server.source_destination_dict.keys()
            )
            self.tcp_server.send_unity_error(error_msg)
            rospy.logerr(error_msg)
            # TODO: send a response to Unity anyway?
            return
        else:
            ros_communicator = self.tcp_server.source_destination_dict[destination]
            service_thread = threading.Thread(
                target=self.service_call_thread, args=(srv_id, destination, data, ros_communicator)
            )
            service_thread.daemon = True
            service_thread.start()

    def service_call_thread(self, srv_id, destination, data, ros_communicator):
        response = ros_communicator.send(data)

        if not response:
            error_msg = "No response data from service '{}'!".format(destination)
            self.tcp_server.send_unity_error(error_msg)
            rospy.logerr(error_msg)
            # TODO: send a response to Unity anyway?
            return

        self.tcp_server.unity_tcp_sender.send_ros_service_response(srv_id, destination, response)

    def run(self):
        """
        Read a message and determine where to send it based on the source_destination_dict
         and destination string. Then send the read message.

        If there is a response after sending the serialized data, assume it is a
        ROS service response.

        Message format is expected to arrive as
            int: length of destination bytes
            str: destination. Publisher topic, Subscriber topic, Service name, etc
            int: size of full message
            msg: the ROS msg type as bytes

        """
        rospy.loginfo("Connection from {}".format(self.incoming_ip))
        halt_event = threading.Event()
        self.tcp_server.unity_tcp_sender.start_sender(self.conn, halt_event)
        try:
            while not halt_event.is_set():
                destination, data = self.read_message(self.conn)

                if self.tcp_server.pending_srv_id is not None:
                    # if we've been told that the next message will be a service request/response, process it as such
                    if self.tcp_server.pending_srv_is_request:
                        self.send_ros_service_request(
                            self.tcp_server.pending_srv_id, destination, data
                        )
                    else:
                        self.tcp_server.send_unity_service_response(
                            self.tcp_server.pending_srv_id, data
                        )
                    self.tcp_server.pending_srv_id = None
                elif destination == "":
                    # ignore this keepalive message, listen for more
                    pass
                elif destination.startswith("__"):
                    # handle a system command, such as registering new topics
                    self.tcp_server.handle_syscommand(destination, data)
                elif destination in self.tcp_server.source_destination_dict:
                    ros_communicator = self.tcp_server.source_destination_dict[destination]
                    response = ros_communicator.send(data)
                else:
                    error_msg = "Topic '{}' is not registered! Known topics are: {} ".format(
                        destination, self.tcp_server.source_destination_dict.keys()
                    )
                    self.tcp_server.send_unity_error(error_msg)
                    rospy.logerr(error_msg)
        except IOError as e:
            rospy.logerr("Exception: {}".format(e))
        finally:
            halt_event.set()
            self.conn.close()
            rospy.loginfo("Disconnected from {}".format(self.incoming_ip))
