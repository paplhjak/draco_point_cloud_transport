#ifndef DRACO_POINT_CLOUD_TRANSPORT_DRACO_PUBLISHER_H
#define DRACO_POINT_CLOUD_TRANSPORT_DRACO_PUBLISHER_H

#include "point_cloud_transport/simple_publisher_plugin.h"
#include "draco_point_cloud_transport/CompressedPointCloud2.h"
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <draco_point_cloud_transport/DracoPublisherConfig.h>

namespace draco_point_cloud_transport {

class DracoPublisher : public point_cloud_transport::SimplePublisherPlugin<draco_point_cloud_transport::CompressedPointCloud2>
{
public:
  virtual ~DracoPublisher() {}

  virtual std::string getTransportName() const
  {
    return "draco";
  }

protected:
  // Overridden to set up reconfigure server
  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const point_cloud_transport::SubscriberStatusCallback  &user_connect_cb,
                             const point_cloud_transport::SubscriberStatusCallback  &user_disconnect_cb,
                             const ros::VoidPtr &tracked_object, bool latch);
  
  virtual void publish(const sensor_msgs::PointCloud2& message,
                       const PublishFn& publish_fn) const;

  typedef draco_point_cloud_transport::DracoPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void configCb(Config& config, uint32_t level);

  std::string base_topic_;
};

} //namespace draco_point_cloud_transport

#endif // DRACO_POINT_CLOUD_TRANSPORT_DRACO_PUBLISHER_H