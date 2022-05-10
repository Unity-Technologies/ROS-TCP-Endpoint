#include <ros/ros.h>
#include <sstream>
#include "../include/ServerThread.h"
#include "../include/ServiceClientThread.h"
#include "../include/EndpointShapeShifter.h"

void* ServiceClientThread(void* args_in)
{
    ServiceClientThreadArgs* args = (ServiceClientThreadArgs*) args_in;
    std::cout << "Started service thread " << args->topicName << std::endl;

    ros::NodeHandle nh;
    ros::ServiceClientOptions options(
        args->topicName,
        args->md5sum,
        false, // persistent
        ros::M_string() // additional args
    );
    ros::ServiceClient client = nh.serviceClient(options);

    EndpointShapeShifter requestShapeShifter;
    EndpointShapeShifter responseShapeShifter;
    requestShapeShifter.morph(args->md5sum, std::string(), std::string(), std::string());
    responseShapeShifter.morph(args->md5sum, std::string(), std::string(), std::string());

    std::vector<uint8_t> responseBuffer;
    std::stringstream ss;

    while(ros::ok() && !args->shouldHalt)
    {
        ServiceClientRequest request = args->requests.awaitDequeue();

        ros::serialization::OStream stream( request.message.data(), request.message.size() );
        requestShapeShifter.read(stream);
        client.call(requestShapeShifter, responseShapeShifter);

        // compose the response syscommand and response message into a single buffer
        responseBuffer.clear();
        SerializeWithLength("__response", responseBuffer);
        SerializeWithLength(request.srvId, responseBuffer);
        SerializeWithLength(args->topicName, responseBuffer);
        SerializeWithLength(responseShapeShifter.get_buffer(), responseShapeShifter.size(), responseBuffer);
        SendToUnity(args->connfd, std::move(responseBuffer));
    }
    delete args;
    return nullptr;
}
