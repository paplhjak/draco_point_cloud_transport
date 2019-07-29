#ifndef DRACO_POINT_CLOUD_TRANSPORT_DRACO_SUBSCRIBER_H
#define DRACO_POINT_CLOUD_TRANSPORT_DRACO_SUBSCRIBER_H

#include "point_cloud_transport/simple_subscriber_plugin.h"
#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <draco_point_cloud_transport/DracoSubscriberConfig.h>

namespace draco_point_cloud_transport {

class DracoSubscriber : public point_cloud_transport::SimpleSubscriberPlugin<draco_point_cloud_transport::CompressedPointCloud2>
{
public:
  virtual ~DracoSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "draco";
  }

  virtual void shutdown();

protected:
  // Overridden to set up reconfigure server
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
          const Callback& callback, const ros::VoidPtr& tracked_object,
          const point_cloud_transport::TransportHints& transport_hints);

  virtual void internalCallback(const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& message,
                                const Callback& user_cb);

  typedef draco_point_cloud_transport::DracoSubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void configCb(Config& config, uint32_t level);
};

} //namespace draco_point_cloud_transport

#endif // DRACO_POINT_CLOUD_TRANSPORT_DRACO_SUBSCRIBER_H