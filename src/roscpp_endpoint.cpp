#include "ros/ros.h"

#include <arpa/inet.h>
#include <sys/socket.h>

#include "../include/SafeQueue.h"
#include "../include/EndpointShapeShifter.h"
#include "../include/ConnectionThread.h"
#include "../include/ServerThread.h"

SafeQueue<PendingNetworkWrite> networkWriteQueue;

int main(int argc, char **argv)
{
    SafeQueue<std::vector<char>> testQueue;
    std::vector<char> testVec;
    testVec.push_back('x');
        printf("Original at %p\n", &testVec[0]);
    testQueue.enqueue(testVec);
    std::vector<char> testResult = testQueue.awaitDequeue();
    printf("Original at %p, result at %p\n", &testVec[0], &testResult[0]);
/*    MessageTypeInfo::Preregister<std_msgs::String>("std_msgs/String");
    MessageTypeInfo::Preregister<sensor_msgs::Imu>("sensor_msgs/Imu");
    MessageTypeInfo::Preregister<geometry_msgs::Vector3>("geometry_msgs/Vector3");
    MessageTypeInfo::Preregister<sensor_msgs::JointState>("sensor_msgs/JointState");
    MessageTypeInfo::Preregister<nav_msgs::Odometry>("nav_msgs/Odometry");
    MessageTypeInfo::Preregister<rosgraph_msgs::Clock>("rosgraph_msgs/Clock");
    MessageTypeInfo::Preregister<sensor_msgs::LaserScan>("sensor_msgs/LaserScan");
    MessageTypeInfo::Preregister<sensor_msgs::Image>("sensor_msgs/Image");
    MessageTypeInfo::Preregister<tf2_msgs::TFMessage>("tf2_msgs/TFMessage");*/

    ros::init(argc, argv, "roscpp_endpoint");
    ros::start();

    pthread_t serverThread;
    pthread_create(&serverThread, NULL, ServerThread, NULL);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        loop_rate.sleep();

        // process subscriber callbacks
        ros::spinOnce();

        // send other network data
        PendingNetworkWrite toWrite;
        while(networkWriteQueue.tryDequeue(&toWrite))
        {
            //printf("Sending queued network msg %s %s\n", toWrite.topic.c_str(), toWrite.message.c_str());
            write(toWrite.connfd, toWrite.buffer.data(), toWrite.buffer.size());
        }
    }
    exit(0);
}

// Our rule is that only the main thread is allowed to write directly to the network.
// Other threads have to call SendToUnity to add their message to the queue (it will be dequeued and written out from the main thread).
void SendToUnity(int connfd, std::string topicName, std::string message)
{
    PendingNetworkWrite pending;
    pending.connfd = connfd;
    SerializeWithLength(topicName, pending.buffer);
    SerializeWithLength(message, pending.buffer);
    networkWriteQueue.enqueue(std::move(pending));
}

void SendToUnity(int connfd, std::vector<uint8_t> buffer)
{
    PendingNetworkWrite pending;
    pending.connfd = connfd;
    pending.buffer = std::move(buffer);
    networkWriteQueue.enqueue(std::move(pending));
}