#include "ros/ros.h"

#include <map>
#include <chrono>
#include <thread>

#include "../include/json.hpp"
#include "../include/EndpointShapeShifter.h"
#include "../include/MessageTypeInfo.h"
#include "../include/ConnectionThread.h"
#include "../include/ServiceClientThread.h"
#include "../include/UnityServiceThread.h"

// caller guarantees write_buffer can actually hold bytes_remaining
bool read_bytes(int connfd, uint8_t* write_buffer, uint bytes_remaining)
{
    while (bytes_remaining > 0)
    {
        ssize_t read_result = read(connfd, write_buffer, bytes_remaining);
        if (read_result < 0)
        {
            std::cerr << "Error on read!" << std::endl;
            exit(0);
        }
        else if (read_result == 0)
        {
            std::cout << "Connection lost" << std::endl;
            return false;
        }
        else
        {
            write_buffer += read_result;
            bytes_remaining -= read_result;
        }
    }

    return true;
}

void* ConnectionThread(void* args_in)
{
    ConnectionThreadArgs* args = (ConnectionThreadArgs*)args_in;

    std::map<std::string, ros::Publisher> publishersTable;
    std::map<std::string, ros::Subscriber> subscribersTable;
    std::map<std::string, ServiceClientThreadArgs*> serviceClientsTable;
    std::map<std::string, UnityServiceThreadArgs*> unityServicesTable;

    int connfd = args->connfd;
    std::string threadName = args->name;

    const int name_buffer_size = 1000;
    const int command_buffer_size = 4000;

    uint8_t name_buff[name_buffer_size];
    uint8_t command_buff[command_buffer_size];
    int n;

    EndpointShapeShifter shape_shifter;

    std::chrono::system_clock::time_point lastWrapReport;
    std::chrono::system_clock::time_point startupTime;
    double smoothedByteRate;
    unsigned long long bytesRead = 0;

    SendToUnity(
        connfd,
        "__handshake",
        "{\"version\":\"v0.7.1c\", \"metadata\":\"{\\\"protocol\\\":\\\"ROS1\\\"}\"}"
    );

    while(true)
    {
        int namesize;
        // read command name size
        if (!read_bytes(connfd, (uint8_t*)&namesize, 4))
            break;

        if(namesize >= name_buffer_size)
        {
            std::cerr << "ABORT: Message name was too large! (Receiving " << namesize << " bytes, limit is " << name_buffer_size << ")" << std::endl;
            exit(1);
        }

        // read command name
        if(!read_bytes(connfd, name_buff, namesize))
            break;
        name_buff[namesize] = 0;//null terminate

        int msgsize;
        if (!read_bytes(connfd, (uint8_t*)&msgsize, 4))
            break;

/*        bytesRead += msgsize + namesize + 8;

        std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
        std::chrono::duration<float> seconds = currentTime - lastWrapReport;
        if(seconds.count() >= 1)
        {
            //float byteRate = NumWraps/(seconds.count()/BufferSize);
            smoothedByteRate = smoothedByteRate * 0.9 + bytesRead * 0.1;
            printf("Throughput: %lf Mbit/sec\n", smoothedByteRate*0.000008);
            lastWrapReport = currentTime;
            bytesRead = 0;
        }*/

        if (namesize == 0)
        {
            //std::cout << threadName << ": Ignoring keepalive" << std::endl;
            continue;
        }

        if (name_buff[0] == '_' && name_buff[1] == '_')
        {
            std::string commandName((char*)name_buff);

            if(msgsize >= command_buffer_size)
            {
                std::cerr << "ABORT: Syscommand '" << name_buff << "' is too large! (Receiving " << msgsize << " bytes, limit is " << command_buffer_size << ")" << std::endl;
                exit(1);
            }
            // It's a syscommand: read the command from the client
            if (!read_bytes(connfd, command_buff, msgsize))
                break;
            command_buff[msgsize] = 0;

            switch(hash(commandName))
            {
                case "__publish"_hash:
                {
                    nlohmann::json j = nlohmann::json::parse((char*)command_buff);

                    std::string topicName = j["topic"].get<std::string>();
                    std::string messageName = j["message_name"].get<std::string>();
                    MessageTypeInfo* info = MessageTypeInfo::Get(messageName);
                    if(info == nullptr)
                    {
                        std::cout << threadName << ": ERROR: Can't advertise topic " << topicName << " - unknown message type " << messageName << std::endl;
                    }
                    else
                    {
                        ros::NodeHandle n;
                        ros::AdvertiseOptions advopt(
                            topicName,
                            10,
                            info->md5sum,
                            info->datatype,
                            info->definition
                        );
                        ros::Publisher publisher = n.advertise(advopt);
                        publishersTable.insert({ topicName, publisher });
                        std::cout << threadName << ": Advertised topic " << topicName << " of type " << messageName << std::endl;
                    }
                    break;
                }

                case "__subscribe"_hash:
                {
                    nlohmann::json j = nlohmann::json::parse((char*)command_buff);

                    std::string topicName = j["topic"].get<std::string>();

                    ros::NodeHandle nh;
                    auto callback = [connfd, topicName](const EndpointShapeShifter::ConstPtr& msg)
                    {
                        SubscriberCallback(connfd, topicName, msg);
                    };
                    ros::Subscriber sub = nh.subscribe<EndpointShapeShifter>(topicName, 1000, callback);
                    subscribersTable.insert({ topicName, sub });
                    std::cout << threadName << ": Subscribed to topic " << topicName << std::endl;
                    break;
                }

                case "__topic_list"_hash:
                {
                    ros::master::V_TopicInfo topic_infos;
                    ros::master::getTopics(topic_infos);

                    std::vector<std::string> topicNames;
                    std::vector<std::string> typeNames;
                    for(auto iter = topic_infos.begin(); iter != topic_infos.end(); ++iter)
                    {
                        topicNames.push_back(iter->name);
                        typeNames.push_back(iter->datatype);
                    }
                    nlohmann::json j = { {"topics", topicNames}, {"types", typeNames} };
                    SendToUnity(connfd, "__topic_list", j.dump());
                    break;
                }

                case "__ping"_hash:
                {
                    // for a ping, we just send back the request
                    SendToUnity(connfd, "__ping_response", (char*)command_buff);
                    break;
                }

                case "__ros_service"_hash:
                {
                    nlohmann::json j = nlohmann::json::parse((char*)command_buff);

                    std::string topicName = j["topic"].get<std::string>();
                    std::string messageName = j["message_name"].get<std::string>();

                    if(serviceClientsTable.find(topicName) != serviceClientsTable.end())
                    {
                        std::cout << threadName << ": WARNING: Ignoring duplicate service registration for " << topicName << std::endl;
                        break;
                    }

                    MessageTypeInfo* info = MessageTypeInfo::GetService(messageName);
                    if(info == nullptr)
                    {
                        std::cout << threadName << ": ERROR: Can't access service " << topicName << " - unknown service type " << messageName << std::endl;
                        break;
                    }
                    ServiceClientThreadArgs* threadArgs = new ServiceClientThreadArgs();
                    threadArgs->connfd = connfd;
                    threadArgs->shouldHalt = false;
                    threadArgs->topicName = topicName;
                    threadArgs->md5sum = info->md5sum;
                    pthread_t connectionThread;
                    pthread_create(&connectionThread, NULL, ServiceClientThread, threadArgs);
                    serviceClientsTable.insert({topicName, threadArgs});

                    break;
                }

                case "__unity_service"_hash:
                {
                    nlohmann::json j = nlohmann::json::parse((char*)command_buff);

                    std::string topicName = j["topic"].get<std::string>();
                    std::string messageName = j["message_name"].get<std::string>();

                    if(unityServicesTable.find(topicName) != unityServicesTable.end())
                    {
                        std::cout << threadName << ": WARNING: Ignoring duplicate service implementation for " << topicName << std::endl;
                        break;
                    }

                    MessageTypeInfo* info = MessageTypeInfo::GetService(messageName);
                    if(info == nullptr)
                    {
                        std::cout << threadName << ": ERROR: Can't implement service " << topicName << " - unknown service type " << messageName << std::endl;
                        break;
                    }
                    UnityServiceThreadArgs* threadArgs = new UnityServiceThreadArgs();
                    threadArgs->connfd = connfd;
                    threadArgs->shouldHalt = false;
                    threadArgs->topicName = topicName;
                    threadArgs->md5sum = info->md5sum;
                    threadArgs->serviceMessageName = messageName;
                    pthread_t connectionThread;
                    pthread_create(&connectionThread, NULL, UnityServiceThread, threadArgs);
                    unityServicesTable.insert({topicName, threadArgs});

                    break;
                }

                case "__request"_hash:
                {
                    std::string requestSrvId((char*)command_buff); // save the command string, we'll send it back with the response

                    if (!read_bytes(connfd, (uint8_t*)&namesize, 4))
                        break;
                    if(!read_bytes(connfd, name_buff, namesize))
                        break;
                    name_buff[namesize] = 0;//null terminate

                    std::string topicName((char*)name_buff);

                    if (!read_bytes(connfd, (uint8_t*)&msgsize, 4))
                        break;
                    std::vector<uint8_t> requestBuffer;
                    requestBuffer.resize(msgsize);
                    if (!read_bytes(connfd, requestBuffer.data(), msgsize))
                        break;

                    auto it = serviceClientsTable.find(topicName);
                    if(it != serviceClientsTable.end())
                    {
                        std::cout << "Handling service request " << topicName << std::endl;
                        it->second->requests.enqueue(ServiceClientRequest(requestSrvId, std::move(requestBuffer)));
                    }
                    else
                    {
                        std::cerr << "Unknown service request " << topicName << std::endl;
                    }

                    break;
                }

                case "__response"_hash:
                {
                    std::string responseSrvId((char*)command_buff); // save the command string

                    if (!read_bytes(connfd, (uint8_t*)&namesize, 4))
                        break;
                    if(!read_bytes(connfd, name_buff, namesize))
                        break;
                    name_buff[namesize] = 0;//null terminate

                    std::string topicName((char*)name_buff);

                    if (!read_bytes(connfd, (uint8_t*)&msgsize, 4))
                        break;
                    std::vector<uint8_t> responseBuffer;
                    responseBuffer.resize(msgsize);
                    if (!read_bytes(connfd, responseBuffer.data(), msgsize))
                        break;

                    auto it = unityServicesTable.find(topicName);
                    if(it != unityServicesTable.end())
                    {
                        std::cout << "Handling service response " << topicName << std::endl;
                        it->second->responses.enqueue(UnityServiceResponse(responseSrvId, std::move(responseBuffer)));
                    }
                    else
                    {
                        std::cerr << "Unknown service response " << topicName << std::endl;
                    }
                    break;
                }

                default:
                {
                    std::cout << threadName << ": WARNING: ignored a syscommand "<< name_buff <<" (TODO: handle this)" << std::endl;
                    break;
                }
            }
        }
        else
        {
            auto it = publishersTable.find(std::string((char*)name_buff));
            if (it == publishersTable.end())
            {
                std::cerr << threadName << " ABORT: No registered publisher for \"" << name_buff << "\"\n";
                break;
            }
            else
            {
                uint8_t* msgbuffer = shape_shifter.prep_buffer(msgsize);

                // read the message from client and copy it in buffer
                if (!read_bytes(connfd, msgbuffer, msgsize))
                    break;

                it->second.publish(shape_shifter);
            }
        }
    }
    for(auto it : serviceClientsTable)
    {
        it.second->shouldHalt = true;
        it.second->requests.interruptWaiters();
    }
    for(auto it : unityServicesTable)
    {
        it.second->shouldHalt = true;
        it.second->responses.interruptWaiters();
    }
    std::cout << threadName << ": hanging up" << std::endl;
    delete args;
    return nullptr;
}
