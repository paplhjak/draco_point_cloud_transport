#include "draco_point_cloud_transport/draco_publisher.h"

#include <boost/make_shared.hpp>

#include "draco_point_cloud_transport/draco_common.h"
#include "draco_point_cloud_transport/conversion_utilities.h"
#include "draco_point_cloud_transport/PC2toDraco.h"

// draco library
#include <draco/compression/expert_encode.h>
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

  base_topic_ = base_topic;
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
    std::unique_ptr<draco::PointCloud> pc = converter.convert(config_.deduplicate, config_.expert_encoding);
    draco::EncoderBuffer encode_buffer;

    // tracks if all necessary parameters were set for expert encoder
    bool expert_settings_ok = true;

    // expert encoder
    if (config_.expert_encoding) {

        draco::ExpertEncoder expert_encoder(*pc);
        expert_encoder.SetSpeedOptions(config_.encode_speed, config_.decode_speed);

        // default
        if((config_.encode_method==0) && (!config_.force_quantization))
        {
            // let draco handle method selection
        }
            // force kd tree
        else if ((config_.encode_method==1) || (config_.force_quantization))
        {
            if(config_.force_quantization)
            {
                // keep track of which attribute is being processed
                int att_id = 0;
                int attribute_quantization_bits;

                for (sensor_msgs::PointField field : message.fields)
                {
                    if (ros::param::get(base_topic_ + "/draco/attribute_mapping/quantization_bits/" + field.name, attribute_quantization_bits))
                    {
                        expert_encoder.SetAttributeQuantization(att_id, attribute_quantization_bits);
                    }
                    else
                    {
                        ROS_ERROR_STREAM ("Attribute type not specified for " + field.name + "field entry.");
                        expert_settings_ok = false;
                    }
                    att_id++;
                }

            }
            expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
        }
            // force sequential encoding
        else {
            expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
        }

        // encodes point cloud and raises error if encoding fails
        //draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
        draco::Status status = expert_encoder.EncodeToBuffer(&encode_buffer);

        if (status.code() != 0)
        {
            ROS_ERROR_STREAM (status);
        }
    }
    // expert encoder end

    // regular encoder
    if ((!config_.expert_encoding) || (!expert_settings_ok))
    {
        draco::Encoder encoder;
        encoder.SetSpeedOptions(config_.encode_speed, config_.decode_speed);

        // default
        if((config_.encode_method==0) && (!config_.force_quantization))
        {
            // let draco handle method selection
        }
            // force kd tree
        else if ((config_.encode_method==1) || (config_.force_quantization))
        {
            if(config_.force_quantization)
            {
                encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, config_.quantization_POSITION);
                encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, config_.quantization_NORMAL);
                encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, config_.quantization_COLOR);
                encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, config_.quantization_TEX_COORD);
                encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, config_.quantization_GENERIC);
            }
            encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
        }
            // force sequential encoding
        else {
            encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
        }

        // encodes point cloud and raises error if encoding fails
        //draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
        draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

        if (status.code() != 0)
        {
            ROS_ERROR_STREAM (status);
        }
    }
    // regular encoder end

    uint32_t compressed_data_size = encode_buffer.size();
    unsigned char* cast_buffer = (unsigned char*)encode_buffer.data();
    std::vector <unsigned char> vec_data(cast_buffer, cast_buffer + compressed_data_size);
    compressed.compressed_data = vec_data;

    publish_fn(compressed);
}

} //namespace draco_point_cloud_transport
