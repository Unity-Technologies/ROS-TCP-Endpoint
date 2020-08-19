#!/usr/bin/env python

import struct
from io import BytesIO

from threading import Thread

from tcp_endpoint.TCPEndpointExceptions import TopicOrServiceNameDoesNotExistError


class ClientThread(Thread):
    """
    Thread class to read all data from a connection and pass along the data to the
    desired source.
    """
    def __init__(self, conn, source_destination_dict):
        """
        Set class variables
        Args:
            conn:
            source_destination_dict: dictionary of destination name to RosCommunicator class
        """
        Thread.__init__(self)
        self.conn = conn
        self.source_destination_dict = source_destination_dict

    def read_int32(self):
        """
        Reads four bytes from socket connection and unpacks them to an int

        Returns: int

        """
        try:
            raw_bytes = self.conn.recv(4)
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

            str_bytes = self.conn.recv(str_len)
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
        data = b''

        destination = self.read_string()
        full_message_size = self.read_int32()

        while len(data) < full_message_size:
            # Only grabs max of 1024 bytes TODO: change to TCPServer's buffer_size
            grab = 1024 if full_message_size - len(data) > 1024 else full_message_size - len(data)
            packet = self.conn.recv(grab)

            if not packet:
                print("No packets...")
                break

            data += packet

        if not data:
            print("No data for a message size of {}, breaking!".format(full_message_size))
            return

        if destination not in self.source_destination_dict.keys():
            error_msg = "Topic/Service destination '{}' does not exist in source destination dictionary {} "\
                .format(destination, self.source_destination_dict.keys())
            self.conn.close()
            raise TopicOrServiceNameDoesNotExistError(error_msg)

        try:
            ros_communicator = self.source_destination_dict[destination]
            response = ros_communicator.send(data)

            # Responses only exist for services
            if response:
                response_message = self.serialize_message(destination, response)
                self.conn.send(response_message)
        except Exception as e:
            print("Exception Raised: {}".format(e))
        finally:
            self.conn.close()
