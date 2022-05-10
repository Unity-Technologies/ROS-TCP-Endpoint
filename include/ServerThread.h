#pragma once

#include <string>
#include "../include/EndpointShapeShifter.h"

void* ServerThread(void* args);

void SerializeWithLength(std::string str, std::vector<uint8_t>& outBuffer);
void SerializeWithLength(const uint8_t* data, unsigned int size, std::vector<uint8_t>& outBuffer);
void SendToUnity(int connfd, std::string topicName, std::string message);
void SendToUnity(int connfd, std::vector<uint8_t> buffer);
void SubscriberCallback(int connfd, std::string topicName, const EndpointShapeShifter::ConstPtr& msg);

struct PendingNetworkWrite
{
    int connfd;
    std::vector<uint8_t> buffer;
};