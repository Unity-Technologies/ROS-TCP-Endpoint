#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "../include/UnityServiceThread.h"
#include "../include/EndpointShapeShifter.h"
#include "../include/EndpointServiceCallbackHelper.h"

void* UnityServiceThread(void* args_in)
{
    UnityServiceThreadArgs* args = (UnityServiceThreadArgs*)args_in;
    ros::NodeHandle nh;
    ros::CallbackQueue cbQueue;
    ros::AdvertiseServiceOptions options;
    options.service = args->topicName;
    options.md5sum = args->md5sum;
    options.callback_queue = &cbQueue;
    options.helper = boost::make_shared<EndpointServiceCallbackHelper>(args);
    options.datatype = args->serviceMessageName;
    options.req_datatype = args->serviceMessageName+"Request";
    options.res_datatype = args->serviceMessageName+"Response";
    ros::ServiceServer service = nh.advertiseService(options);

    while(ros::ok() && !args->shouldHalt)
    {
        cbQueue.callAvailable(ros::WallDuration(1));
    }

    service.shutdown();

    std::cout << "Ending unity service thread " << args->topicName << std::endl;
    delete args;
    return nullptr;
}
