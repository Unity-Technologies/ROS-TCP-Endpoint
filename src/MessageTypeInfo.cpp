//#define PY_SSIZE_T_CLEAN
//#include <Python.h>
#include "../include/MessageTypeInfo.h"
#include <ros/package.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>

std::map<std::string, MessageTypeInfo*> MessageTypeInfo::messageTypesCache;

MessageTypeInfo* MessageTypeInfo::Get(std::string messageName, bool isService)
{
    auto it = messageTypesCache.find(messageName);
    if (it != messageTypesCache.end())
    {
        return it->second;
    }

    // if not in memory, check our cache on disk
    std::string path = ros::package::getPath("ros_tcp_endpoint");
    std::string cacheFilePath = path + "/type_cache/" + messageName;
    std::string md5sum;
    std::string definition;

    std::ifstream cachedFile(cacheFilePath);

    if(cachedFile.good())
    {
        std::getline(cachedFile, md5sum);
        std::stringstream buffer;
        buffer << cachedFile.rdbuf();
        definition = buffer.str();

        MessageTypeInfo* result = new MessageTypeInfo(md5sum, messageName, definition);
        messageTypesCache.insert({messageName, result});
        return result;
    }

    // If not on disk, run a command line executable to get message type info.
    // Awkward and slow, but we'll cache it so we don't need to run it again
    std::string commandName = isService? "rossrv": "rosmsg";
    std::string md5Command = commandName + " md5 " + messageName;
    FILE* fp = popen(md5Command.c_str(), "r");
    if(fp == nullptr)
    {
        std::cerr << "Failed to run command '" << md5Command << "'" << std::endl;
        return nullptr;
    }
    char buffer[1000];
    if(fgets(buffer, sizeof(buffer), fp) != nullptr)
    {
        md5sum.append(buffer);
        md5sum.erase(std::remove(md5sum.begin(), md5sum.end(), '\n'), md5sum.end());
    }

    if(md5sum.length() == 0)
    {
        messageTypesCache.insert({messageName, nullptr});
        std::cerr << "Failed to load md5 for message '" << messageName << "'" << std::endl;
        return nullptr;
    }

    if(!isService)
    {
        std::string defCommand = commandName + " show " + messageName;
        fp = popen(defCommand.c_str(), "r");
        if(fp == nullptr)
        {
            std::cerr << "Failed to run command '" << defCommand << "'" << std::endl;
            return nullptr;
        }
        while(!feof(fp))
        {
            if(fgets(buffer, sizeof(buffer), fp) != nullptr)
            {
                definition.append(buffer);
            }
        }
        if(definition.length() == 0)
        {
            messageTypesCache.insert({messageName, nullptr});
            std::cerr << "Failed to load definition for message '" << messageName << "'" << std::endl;
            return nullptr;
        }
    }

    // Write it to a file so we won't have to run that command again
    std::string cacheBasePath = path + "/type_cache";
    mkdir(cacheBasePath.c_str(), 0777);

    size_t separatorIdx = messageName.find('/');
    std::string packageName = messageName.substr(0, separatorIdx);
    std::string cachePackagePath = cacheBasePath + "/" + packageName;
    mkdir(cachePackagePath.c_str(), 0777);

    std::ofstream file;
    file.open(cacheFilePath);
    file << md5sum << std::endl;
    file << definition;
    file.close();

    // Finally, add it to our dictionary and return
    MessageTypeInfo* result = new MessageTypeInfo(md5sum, messageName, definition);
    messageTypesCache.insert({messageName, result});

    return result;
}