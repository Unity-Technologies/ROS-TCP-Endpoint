/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include "ros/assert.h"
#include <vector>
#include <string>
#include <string.h>

#include <ros/message_traits.h>
//#include "macros.h"
 
class ShapeShifterException : public ros::Exception
{
public:
    ShapeShifterException(const std::string& msg)
        : ros::Exception(msg)  {}
};

// Based on the topic_tools::ShapeShifter message, but modified to allow direct access to the message buffer
class EndpointShapeShifter
{
public:
    typedef boost::shared_ptr<EndpointShapeShifter> Ptr;
    typedef boost::shared_ptr<EndpointShapeShifter const> ConstPtr;

    static bool uses_old_API_;

    // Constructor and destructor
    EndpointShapeShifter();
    virtual ~EndpointShapeShifter();

    // Helpers for inspecting shapeshifter
    std::string const& getDataType()          const;
    std::string const& getMD5Sum()            const;
    std::string const& getMessageDefinition() const;

    void morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def, const std::string& latching);

    // Helper for advertising
    ros::Publisher advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch=false, 
                              const ros::SubscriberStatusCallback &connect_cb=ros::SubscriberStatusCallback()) const;

    //! Call to try instantiating as a particular type
    template<class M> 
    boost::shared_ptr<M> instantiate() const;

    //! Write serialized message contents out to a stream
    template<typename Stream>
    void write(Stream& stream) const;

    template<typename Stream>
    void read(Stream& stream);

    //! Return the size of the serialized message
    uint32_t size() const;

    //New in Roscpp_endpoint:
    const uint8_t* get_buffer() const
    {
        return msgBuf.data();
    }

    uint8_t* prep_buffer(size_t length)
    {
        msgBuf.resize(length);
        return msgBuf.data();
    }

    // so it can be used as a service request/response
    std::string const& __getServerMD5Sum() const
    {
        return getMD5Sum();
    }

    private:

    std::string md5, datatype, msg_def, latching;
    bool typed;

    std::vector<uint8_t> msgBuf;
};


// Message traits allow shape shifter to work with the new serialization API
namespace ros
{
    namespace message_traits
    {
        template <> struct IsMessage<EndpointShapeShifter> : TrueType { };
        template <> struct IsMessage<const EndpointShapeShifter> : TrueType { };

        template<>
        struct MD5Sum<EndpointShapeShifter>
        {
            static const char* value(const EndpointShapeShifter& m) { return m.getMD5Sum().c_str(); }

            // Used statically, a shapeshifter appears to be of any type
            static const char* value() { return "*"; }
        };

        template<>
        struct DataType<EndpointShapeShifter>
        {
            static const char* value(const EndpointShapeShifter& m) { return m.getDataType().c_str(); }

            // Used statically, a shapeshifter appears to be of any type
            static const char* value() { return "*"; }
        };

        template<>
        struct Definition<EndpointShapeShifter>
        {
            static const char* value(const EndpointShapeShifter& m) { return m.getMessageDefinition().c_str(); }
        };
    } // namespace message_traits

    namespace serialization
    {
        template<>
        struct Serializer<EndpointShapeShifter>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const EndpointShapeShifter& m)
            {
                m.write(stream);
            }

            template<typename Stream>
            inline static void read(Stream& stream, EndpointShapeShifter& m)
            {
                m.read(stream);
            }

            inline static uint32_t serializedLength(const EndpointShapeShifter& m) {
                return m.size();
            }
        };

        template<>
        struct PreDeserialize<EndpointShapeShifter>
        {
            static void notify(const PreDeserializeParams<EndpointShapeShifter>& params)
            {
                std::string md5      = (*params.connection_header)["md5sum"];
                std::string datatype = (*params.connection_header)["type"];
                std::string msg_def  = (*params.connection_header)["message_definition"];
                std::string latching  = (*params.connection_header)["latching"];

                params.message->morph(md5, datatype, msg_def, latching);
            }
        };
    } // namespace serialization
} //namespace ros

// Template implementations:

  //
  //  only used in testing, seemingly
  //
template<class M> 
boost::shared_ptr<M> EndpointShapeShifter::instantiate() const
{
    if (!typed)
        throw ShapeShifterException("Tried to instantiate message from an untyped shapeshifter.");

    if (ros::message_traits::datatype<M>() != getDataType())
        throw ShapeShifterException("Tried to instantiate message without matching datatype.");

    if (ros::message_traits::md5sum<M>() != getMD5Sum())
        throw ShapeShifterException("Tried to instantiate message without matching md5sum.");

    boost::shared_ptr<M> p(boost::make_shared<M>());

    // The IStream never modifies its data, and nothing else has access to this
    // object, so the const_cast here is ok
    ros::serialization::IStream s(const_cast<unsigned char*>(msgBuf.data()), msgBuf.size());
    ros::serialization::deserialize(s, *p);

    return p;
}

template<typename Stream>
void EndpointShapeShifter::write(Stream& stream) const
{
    //printf("Shapeshifter write %lu\n", msgBuf.size());
    if (msgBuf.size() > 0)
        memcpy(stream.advance(msgBuf.size()), msgBuf.data(), msgBuf.size());
}

template<typename Stream>
void EndpointShapeShifter::read(Stream& stream)
{
    stream.getLength();
    stream.getData();

    // stash this message in our buffer
    msgBuf.resize(stream.getLength());
    //printf("Shapeshifter read %lu\n", msgBuf.size());
    memcpy(msgBuf.data(), stream.getData(), stream.getLength());
}
