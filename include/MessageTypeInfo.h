#pragma once

#include <string>
#include <map>
#include "ros/message_traits.h"

class MessageTypeInfo
{
    public:
    std::string md5sum;
    std::string datatype;
    std::string definition;

    MessageTypeInfo(std::string md5sum, std::string datatype, std::string definition):
        md5sum(md5sum), datatype(datatype), definition(definition)
    {
    }

    static MessageTypeInfo* Get(std::string messageName) { return Get(messageName, "message_type_info.py"); }
    static MessageTypeInfo* GetService(std::string messageName) { return Get(messageName, "service_type_info.py"); }

    template <typename T>
    static void Preregister(std::string name)
    {
        messageTypesCache.insert({ name, new MessageTypeInfo(
            ros::message_traits::md5sum<T>(),
            ros::message_traits::datatype<T>(),
            ros::message_traits::definition<T>()
        ) });
    }

    private:
    static std::map<std::string, MessageTypeInfo*> messageTypesCache;

    static MessageTypeInfo* Get(std::string messageName, std::string commandName);
};
