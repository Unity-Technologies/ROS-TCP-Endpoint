//#define PY_SSIZE_T_CLEAN
//#include <Python.h>
#include "../include/MessageTypeInfo.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <sstream>

std::map<std::string, MessageTypeInfo*> MessageTypeInfo::messageTypesCache;

MessageTypeInfo* MessageTypeInfo::Get(std::string messageName, std::string commandName)
{
    auto it = messageTypesCache.find(messageName);
    if (it != messageTypesCache.end())
    {
        return it->second;
    }

    // Launch a python instance to get message type info.
    // Awkward and slow, but it only happens once per message type we want to publish
    std::stringstream ss;
    ss << "rosrun roscpp_endpoint " << commandName << " " << messageName;
    FILE* fp = popen(ss.str().c_str(), "r");
    if(fp == nullptr)
    {
        std::cerr << "Failed to run command '" << ss.str() << "'" << std::endl;
        return nullptr;
    }
    char buffer[1000];
    std::string md5sum;
    std::string definition;
    if(fgets(buffer, sizeof(buffer), fp) != nullptr)
    {
        md5sum.append(buffer);
        md5sum.erase(std::remove(md5sum.begin(), md5sum.end(), '\n'), md5sum.end());
    }
    while(!feof(fp))
    {
        if(fgets(buffer, sizeof(buffer), fp) != nullptr)
        {
            definition.append(buffer);
        }
    }
    int returnCode = pclose(fp);
    bool success = (WIFEXITED(returnCode) && WEXITSTATUS(returnCode) == 0);

    if(success)
    {
        MessageTypeInfo* result = new MessageTypeInfo(md5sum, messageName, definition);
        messageTypesCache.insert({messageName, result});
        return result;
    }
    else
    {
        messageTypesCache.insert({messageName, nullptr});
        std::cerr << "Failed to load info for message '" << messageName << "'" << std::endl;
        return nullptr;
    }
}