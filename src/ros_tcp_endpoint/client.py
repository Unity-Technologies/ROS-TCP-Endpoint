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
import select
from io import BytesIO

from threading import Thread

from .exceptions import TopicOrServiceNameDoesNotExistError


class ClientThread(Thread):
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
        Thread.__init__(self)
        self.conn = conn
        self.tcp_server = tcp_server
        self.incoming_ip = incoming_ip
        self.incoming_port = incoming_port

    def read(self, size):
        """
        Reads the given number of bytes from socket connection
        Returns: red bytes
        """
        if self.tcp_server.keep_connections:
            ready = select.select([self.conn], [], [], self.tcp_server.timeout_in_seconds)
            if ready[0]:
                return self.conn.recv(size)
        else:
            return self.conn.recv(size)
            
        return None

    def read_int32(self):
        """
        Reads four bytes from socket connection and unpacks them to an int

        Returns: int

        """
        try:
            raw_bytes = self.read(4)
            num = struct.unpack('<I', raw_bytes)[0]
            return num
        except Exception as e:
            print("Unable to read integer from connection. {}".format(e))

        return None

    def read_string(self):
        """
        Reads int32 from socket connection to determine how many bytes to
        read to get the string that follows. Read that number of bytes and
        decode to utf-8 string.

        Returns: string

        """
        try:
            str_len = self.read_int32()
            str_bytes = self.read(str_len)
            decoded_str = str_bytes.decode('utf-8')
            return decoded_str

        except Exception as e:
            print("Unable to read string from connection. {}".format(e))

        return None

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
        dest_bytes = destination.encode('utf-8')
        length = len(dest_bytes)
        dest_info = struct.pack('<I%ss' % length, length, dest_bytes)

        serial_response = BytesIO()
        message.serialize(serial_response)

        # Per documention, https://docs.python.org/3.8/library/io.html#io.IOBase.seek,
        # seek to end of stream for length
        # SEEK_SET or 0 - start of the stream (the default); offset should be zero or positive
        # SEEK_CUR or 1 - current stream position; offset may be negative
        # SEEK_END or 2 - end of the stream; offset is usually negative
        response_len = serial_response.seek(0, 2)

        msg_length = struct.pack('<I', response_len)
        serialized_message = dest_info + msg_length + serial_response.getvalue()

        return serialized_message

    def run(self):
        """
        Decode destination and full message size from socket connection.
        Grab bytes in chunks until full message has been read.
        Determine where to send the message based on the source_destination_dict
         and destination string. Then send the read message.

        If there is a response after sending the serialized data, assume it is a
        ROS service response.

        Message format is expected to arrive as
            int: length of destination bytes
            str: destination. Publisher topic, Subscriber topic, Service name, etc
            int: size of full message
            msg: the ROS msg type as bytes

        """
        while True:
            data = b''

            destination = self.read_string()
            
            if not destination:
                print("No destination... Maybe the client disconnected")
                self.conn.close()
                return

            full_message_size = self.read_int32()

            while len(data) < full_message_size:
                # Only grabs max of 1024 bytes TODO: change to TCPServer's buffer_size
                grab = 1024 if full_message_size - len(data) > 1024 else full_message_size - len(data)
                packet = self.read(grab)

                if not packet:
                    print("No packets...")
                    break

                data += packet

            if not data:
                print("No data for a message size of {}, breaking!".format(full_message_size))
                self.conn.close()
                return

            if destination == '__syscommand':
                self.tcp_server.handle_syscommand(data)
                if not self.tcp_server.keep_connections:
                    return
                continue
            elif destination == '__handshake':
                response = self.tcp_server.unity_tcp_sender.handshake(self.incoming_ip, data)
                response_message = self.serialize_message(destination, response)
                self.conn.send(response_message)
                if not self.tcp_server.keep_connections:
                    return
                continue
            elif destination not in self.tcp_server.source_destination_dict.keys():
                error_msg = "Topic/service destination '{}' is not defined! Known topics are: {} "\
                    .format(destination, self.tcp_server.source_destination_dict.keys())
                if not self.tcp_server.keep_connections:
                    self.conn.close()
                self.tcp_server.send_unity_error(error_msg)
                raise TopicOrServiceNameDoesNotExistError(error_msg)
            else:
                ros_communicator = self.tcp_server.source_destination_dict[destination]

            try:
                response = ros_communicator.send(data)
                # Responses only exist for services
                if response:
                    response_message = self.serialize_message(destination, response)
                    self.conn.send(response_message)
            except Exception as e:
                print("Exception Raised in client : {}".format(e))
            finally:
                if not self.tcp_server.keep_connections:
                    self.conn.close()
                    return
