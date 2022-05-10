#include <ros/service_callback_helper.h>
#include "../include/ServerThread.h"
#include "../include/UnityServiceThread.h"

class EndpointServiceCallbackHelper : public ros::ServiceCallbackHelper
{
    UnityServiceThreadArgs* m_Args;
    std::vector<uint8_t> m_RequestBuffer;

    public:
    EndpointServiceCallbackHelper(UnityServiceThreadArgs* args):
        m_Args(args)
    {
    }
    virtual ~EndpointServiceCallbackHelper() {}

    virtual bool call(ros::ServiceCallbackHelperCallParams& params)
    {
        m_RequestBuffer.clear();
        SerializeWithLength("__request", m_RequestBuffer);
        SerializeWithLength("{\"srv_id\":100}", m_RequestBuffer);
        SerializeWithLength(m_Args->topicName, m_RequestBuffer);
        SerializeWithLength(params.request.message_start, params.request.num_bytes, m_RequestBuffer);
        SendToUnity(m_Args->connfd, m_RequestBuffer);

        UnityServiceResponse response;
        while(ros::ok() && !m_Args->shouldHalt)
        {
            if(m_Args->responses.tryAwaitDequeue(&response))
            {
                EndpointShapeShifter shapeshifter;
                ros::serialization::OStream stream( response.message.data(), response.message.size() );
                shapeshifter.read(stream);
                params.response = ros::serialization::serializeServiceResponse(true, shapeshifter);
                return true;
            }
        }

        return false;
    }
};