cmake_minimum_required(VERSION 3.0.2)
project(ros_tcp_endpoint)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  #rospy
)

#catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_tcp_endpoint
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(endpoint
    src/roscpp_endpoint.cpp
    src/EndpointShapeShifter.cpp
    src/MessageTypeInfo.cpp
    src/ConnectionThread.cpp
    src/ServerThread.cpp
    src/ServiceClientThread.cpp
    src/UnityServiceThread.cpp)
target_link_libraries(endpoint ${catkin_LIBRARIES})


#catkin_install_python(
#  PROGRAMS src/message_type_info.py src/service_type_info.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

