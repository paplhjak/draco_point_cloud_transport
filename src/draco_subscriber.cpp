#include "draco_point_cloud_transport/draco_subscriber.h"

#include "draco_point_cloud_transport/draco_common.h"
#include "draco_point_cloud_transport/DracotoPC2.h"
#include "draco_point_cloud_transport/conversion_utilities.h"

#include "draco/compression/decode.h"

#include <limits>
#include <vector>

namespace draco_point_cloud_transport
{

void DracoSubscriber::subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const point_cloud_transport::TransportHints& transport_hints)
{
    typedef point_cloud_transport::SimpleSubscriberPlugin<draco_point_cloud_transport::CompressedPointCloud2> Base;
    Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);

    // Set up reconfigure server for this topic
    reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
    ReconfigureServer::CallbackType f = boost::bind(&DracoSubscriber::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
}

void DracoSubscriber::configCb(Config& config, uint32_t level)
{
  config_ = config;
}

void DracoSubscriber::shutdown()
{
  reconfigure_server_.reset();
    point_cloud_transport::SimpleSubscriberPlugin<draco_point_cloud_transport::CompressedPointCloud2>::shutdown();
}

void DracoSubscriber::internalCallback(const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& message,
                                            const Callback& user_cb)

{
    // get size of buffer with compressed data in Bytes
    uint32_t compressed_data_size = message->compressed_data.size();

    // empty buffer
    if (compressed_data_size==0)
    {
        return ;
    }

    draco::DecoderBuffer decode_buffer;
    std::vector <unsigned char> vec_data = (message->compressed_data);

    // Sets the buffer's internal data. Note that no copy of the input data is
    // made so the data owner needs to keep the data valid and unchanged for
    // runtime of the decoder.
    decode_buffer.Init(reinterpret_cast<const char *>(&vec_data[0]), compressed_data_size);

    // create decoder object
    draco::Decoder decoder;
    // set decoder from dynamic reconfiguration
    if(config_.SkipDequantizationPOSITION)
    {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::POSITION);
    }
    if(config_.SkipDequantizationNORMAL)
    {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::NORMAL);
    }
    if(config_.SkipDequantizationCOLOR)
    {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::COLOR);
    }
    if(config_.SkipDequantizationTEX_COORD)
    {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::TEX_COORD);
    }
    if(config_.SkipDequantizationGENERIC)
    {
        decoder.SetSkipAttributeTransform(draco::GeometryAttribute::GENERIC);
    }

    // decode buffer into draco point cloud
    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();


    // create and initiate converter object
    DracotoPC2 converter_b(std::move(decoded_pc), message);
    // convert draco point cloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2Ptr ptr_PC2( new sensor_msgs::PointCloud2( std::move(converter_b.convert()) ) );


    // Publish message to user callback
    user_cb(ptr_PC2);
}

} //namespace draco_point_cloud_transport
