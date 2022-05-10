#pragma once

#include <string>
#include "../include/SafeQueue.h"

struct UnityServiceResponse
{
    std::string srvId;
    std::vector<uint8_t> message;
    UnityServiceResponse()
    {}

    UnityServiceResponse(std::string srvId, std::vector<uint8_t> message):
        srvId(srvId), message(message)
    {
    }
};

struct UnityServiceThreadArgs
{
    int connfd;
    std::string topicName;
    std::string md5sum;
    std::string serviceMessageName;
    SafeQueue<UnityServiceResponse> responses;
    bool shouldHalt;
};

void* UnityServiceThread(void* args_in);
