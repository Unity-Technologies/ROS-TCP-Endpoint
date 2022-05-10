#include <arpa/inet.h>
#include <sys/socket.h>

#include "../include/SafeQueue.h"
#include "../include/ServerThread.h"
#include "../include/ConnectionThread.h"

void SerializeWithLength(std::string str, std::vector<uint8_t>& outBuffer)
{
    unsigned int strSize = str.size();
    outBuffer.insert(outBuffer.end(), (uint8_t*)&strSize, ((uint8_t*)&strSize)+sizeof(unsigned int));
    outBuffer.insert(outBuffer.end(), str.begin(), str.end());
}

void SerializeWithLength(const uint8_t* data, unsigned int size, std::vector<uint8_t>& outBuffer)
{
    outBuffer.insert(outBuffer.end(), (uint8_t*)&size, ((uint8_t*)&size)+sizeof(unsigned int));
    outBuffer.insert(outBuffer.end(), data, data+size);
}

// Since Subscriber callbacks run on the main thread (because that's where we call ros::spin()), they are allowed
// to write directly to the network.
void SubscriberCallback(int connfd, std::string topicName, const EndpointShapeShifter::ConstPtr& msg)
{
    pthread_t self = pthread_self();
    uint32_t topicNameSize = topicName.size();
    write(connfd, &topicNameSize, 4);
    write(connfd, topicName.c_str(), topicNameSize);
    uint32_t msgSize = msg->size();
    write(connfd, &msgSize, 4);
    write(connfd, msg->get_buffer(), msg->size());
}

void* ServerThread(void* args)
{
    int numThreads = 0;
    int sockfd, connfd;
    socklen_t len;
    struct sockaddr_in servaddr, cli;

    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        std::cerr << "ABORT: Cannot create socket!" << std::endl;
        exit(1);
    }
    bzero(&servaddr, sizeof(servaddr));

    ros::NodeHandle nh("~");
    std::string ipAddressParam;
    nh.param<std::string>("tcp_ip", ipAddressParam, "0.0.0.0");
    int portParam;
    nh.param<int>("tcp_port", portParam, 10000);

    std::cout << "Starting server on " << ipAddressParam << ":" << portParam << std::endl;

    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(ipAddressParam.c_str()); //htonl(INADDR_ANY); //inet_addr("127.0.0.1");// 
    servaddr.sin_port = htons(portParam);

    // Binding newly created socket to given IP and verification
    if ((bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) != 0)
    {
        std::cerr << "ABORT: Cannot bind socket!" << std::endl;
        exit(1);
    }

    while (true)
    {
        // Now server is ready to listen and verification
        if ((listen(sockfd, 5)) != 0)
        {
            std::cerr << "ABORT: Socket listen failed!" << std::endl;
            exit(1);
        }

        len = sizeof(cli);

        // Accept the data packet from client and verification
        connfd = accept(sockfd, (struct sockaddr*)&cli, &len);
        if (connfd < 0)
        {
            std::cerr << "ABORT: Socket accept failed!" << std::endl;
            exit(1);
        }
        std::cout << "Connected." << std::endl;

        ConnectionThreadArgs* threadArgs = new ConnectionThreadArgs();
        threadArgs->connfd = connfd;
        threadArgs->name = "T0";
        threadArgs->name[1] += numThreads;
        pthread_t connectionThread;
        pthread_create(&connectionThread, NULL, ConnectionThread, threadArgs);
        numThreads++;
    }
}
