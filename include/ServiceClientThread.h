#pragma once
#include "SafeQueue.h"
#include <vector>

struct ServiceClientRequest
{
    std::string srvId;
    std::vector<uint8_t> message;

    ServiceClientRequest(std::string srvId, std::vector<uint8_t> message):
        srvId(srvId), message(message)
    {
    }
};

struct ServiceClientThreadArgs
{
    int connfd;
    std::string topicName;
    std::string md5sum;
    SafeQueue<ServiceClientRequest> requests;
    bool shouldHalt;
};

void* ServiceClientThread(void* args_in);
