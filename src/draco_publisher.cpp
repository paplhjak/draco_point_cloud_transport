#include "draco_point_cloud_transport/draco_publisher.h"

#include <boost/make_shared.hpp>

#include "draco_point_cloud_transport/draco_common.h"
#include "draco_point_cloud_transport/conversion_utilities.h"
#include "draco_point_cloud_transport/PC2toDraco.h"

// draco library
#include <draco/compression/encode.h>

#include <vector>

namespace draco_point_cloud_transport
{

void DracoPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                        const point_cloud_transport::SubscriberStatusCallback &user_connect_cb,
                                        const point_cloud_transport::SubscriberStatusCallback &user_disconnect_cb,
                                        const ros::VoidPtr &tracked_object, bool latch)
{
  typedef point_cloud_transport::SimplePublisherPlugin<draco_point_cloud_transport::CompressedPointCloud2> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&DracoPublisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void DracoPublisher::configCb(Config& config, uint32_t level)
{
  config_ = config;
}

void DracoPublisher::publish(const sensor_msgs::PointCloud2& message, const PublishFn& publish_fn) const
{
  // Compressed message
    draco_point_cloud_transport::CompressedPointCloud2 compressed;

    assign_description_of_PointCloud2(compressed, message);

    PC2toDraco converter(message);
    std::unique_ptr<draco::PointCloud> pc = converter.convert(config_.deduplicate);
    draco::EncoderBuffer encode_buffer;

    draco::Encoder encoder;

    // set quantization of attribute types
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, config_.quantization_POSITION);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, config_.quantization_NORMAL);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, config_.quantization_COLOR);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, config_.quantization_TEX_COORD);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, config_.quantization_GENERIC);

    // check if encode/decode speed are valid
    if ( (config_.encode_speed <= 10) && (config_.encode_speed >=0) && (config_.decode_speed <= 10) && (config_.decode_speed >=0) )
    {
        encoder.SetSpeedOptions(config_.encode_speed, config_.decode_speed);
    }
    else
    {
        //! RAISE ERROR: invalid encoding/decoding speed
        ROS_ERROR("Draco Point Cloud Transport - compression allows encoding/decoding speed in interval [0,10] (input encode_speed is: %d, input decode_speed is: %d)", config_.encode_speed , config_.decode_speed);
    }

    // TODO: Commented out for as long as sequential encoding is not working
    /*
    // kd tree
    if(config_.method==1)
    {
        encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    }
    // sequential encoding
    else if(config_.method==2)
    {
        encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }
    // draco will choose automatically - default
    else {
    }
    */
    // TODO: For as long as sequential encoding is not working, use KD-tree only
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);

    // encode point cloud to encode_buffer
    encoder.SetTrackEncodedProperties(false);
    encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

    uint32_t compressed_data_size = encode_buffer.size();
    unsigned char* cast_buffer = (unsigned char*)encode_buffer.data();
    std::vector <unsigned char> vec_data(cast_buffer, cast_buffer + compressed_data_size);
    compressed.compressed_data = vec_data;



    publish_fn(compressed);
}

} //namespace draco_point_cloud_transport
