cmake_minimum_required(VERSION 2.8.3)
project(ros_tcp_endpoint)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS message_runtime rospy)

catkin_install_python(PROGRAMS    src/ros_tcp_endpoint/client.py
                                  src/ros_tcp_endpoint/communication.py
                                  src/ros_tcp_endpoint/default_server_endpoint.py
                                  src/ros_tcp_endpoint/exceptions.py
                                  src/ros_tcp_endpoint/publisher.py
                                  src/ros_tcp_endpoint/server.py
                                  src/ros_tcp_endpoint/service.py
                                  src/ros_tcp_endpoint/subscriber.py
                                  src/ros_tcp_endpoint/tcp_sender.py
                                  src/ros_tcp_endpoint/thread_pauser.py
                                  src/ros_tcp_endpoint/unity_service.py
                                  src/ros_tcp_endpoint/default_server_endpoint_venv.py
                                  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES requirements.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})