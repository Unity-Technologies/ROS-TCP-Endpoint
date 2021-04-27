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

import rospy
import socket
import time
import threading
import struct
from .client import ClientThread
from ros_tcp_endpoint.msg import RosUnityError
from ros_tcp_endpoint.msg import RosUnitySrvMessage
from io import BytesIO

# queue module was renamed between python 2 and 3
try:
    from queue import Queue
    from queue import Empty
except:
    from Queue import Queue
    from Queue import Empty

class UnityTcpSender:
    """
    Connects and sends messages to the server on the Unity side.
    """
    def __init__(self, timeout_on_connect, timeout_on_send, timeout_on_idle):
        # if we have a valid IP at this point, it was overridden locally so always use that
        self.timeout_on_connect = timeout_on_connect
        self.timeout_on_send = timeout_on_send
        self.timeout_on_idle = timeout_on_idle
        self.queue = Queue()
        self.sender_id = 1
        
        # variables needed for matching up unity service requests with responses
        self.next_srv_id = 1001
        self.srv_lock = threading.Lock()
        self.services_waiting = {}

    def send_unity_error(self, error):
        self.send_unity_message("__error", RosUnityError(error))

    def send_unity_message(self, topic, message):
        serialized_message = ClientThread.serialize_message(topic, message)
        self.queue.put(serialized_message)

    def send_unity_service(self, topic, service_class, request):
        request_bytes = BytesIO()
        request.serialize(request_bytes)
        result = [None]
        condition = threading.Condition()
        
        with self.srv_lock:
            srv_id = self.next_srv_id
            self.next_srv_id+=1
            self.services_waiting[srv_id] = (condition, result)
        
        payload = request_bytes.getvalue()
        srv_message = RosUnitySrvMessage(srv_id, True, topic, payload)
        serialized_message = ClientThread.serialize_message('__srv', srv_message)
        self.queue.put(serialized_message)

        # rospy starts a new thread for each service request,
        # so it won't break anything if we suspend it while waiting for the response
        with condition:
            condition.wait()
        
        response = service_class._response_class().deserialize(result[0])
        return response
    
    def send_unity_service_response(self, srv_id, data):
        with self.srv_lock:
            (condition, result) = self.services_waiting[srv_id]
            result[0] = data
            del self.services_waiting[srv_id]
        
        with condition:
            condition.notify()

    def start_sender(self, conn, reader_halt, sender_halt):
        sender_thread = threading.Thread(target=self.sender_loop, args=(conn, self.sender_id, reader_halt, sender_halt))
        self.sender_id += 1
        # Exit the server thread when the main thread terminates
        sender_thread.daemon = True
        sender_thread.start()
 
    def sender_loop(self, conn, tid, reader_halt, sender_halt):
        s = None
        
        while True:
            try:
                item = self.queue.get(timeout=0.5)
            except Empty:
                # ideally we'd just wait on the queue, but we also need to check frequently for the connection being closed 
                # (otherwise we might take a message and fail to send it)
                if sender_halt[0]:
                    return
                continue

            if sender_halt[0]:
                return
            
            #print("Sender {} sending an item".format(tid))

            try:
                conn.sendall(item)
            except Exception as e:
                rospy.loginfo("Exception on Send {}".format(e))
                reader_halt[0] = True
                conn.close()
                break
