#include <pluginlib/class_list_macros.h>
#include "draco_point_cloud_transport/draco_publisher.h"
#include "draco_point_cloud_transport/draco_subscriber.h"

PLUGINLIB_EXPORT_CLASS( draco_point_cloud_transport::DracoPublisher, point_cloud_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS( draco_point_cloud_transport::DracoSubscriber, point_cloud_transport::SubscriberPlugin)
