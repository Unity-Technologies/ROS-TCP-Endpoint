cmake_minimum_required(VERSION 2.8.3)
project(ros_tcp_endpoint)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
