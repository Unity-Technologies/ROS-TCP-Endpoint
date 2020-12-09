import rospy
import socket
from tcp_endpoint.RosTCPClientThread import ClientThread
from tcp_endpoint.msg import RosUnityError

class UnityTCPSender:
    """
    Connects and sends messages to the server on the Unity side.
    """
    def __init__(self, unity_ip, unity_port):
        self.unity_ip = unity_ip
        self.unity_port = unity_port
        # if we have a valid IP at this point, it was overridden locally so always use that
        self.ip_is_overridden = (self.unity_ip != '')
        self.keep_connection_open = False
        self.socket = None

    def process_handshake(self, ip, port):
        self.unity_port = port
        if ip != '' and not self.ip_is_overridden:
            self.unity_ip = ip # hello Unity, we'll talk to you from now on
        print("ROS-Unity Handshake received, will connect to {}:{}".format(self.unity_ip, self.unity_port))

    def send_unity_error(self, error):
        self.send_unity_message("__error", RosUnityError(error))

    def send_unity_message(self, topic, message):
        if self.unity_ip == '':
            print("Can't send a message, no defined unity IP!".format(topic, message))
            return

        serialized_message = ClientThread.serialize_message(topic, message)
        try:
            if not self.keep_connection_open or self.socket == None:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.connect((self.unity_ip, self.unity_port))
                if self.keep_connection_open:
                    self.socket = s
                s.send(serialized_message)
            else:
                self.socket.send(serialized_message)
            if not self.keep_connection_open:
                s.close()
        except Exception as e:
            if self.socket != None:
                self.socket.close()
                self.socket = None
            rospy.loginfo("Exception {}".format(e))
