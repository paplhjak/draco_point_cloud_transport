
#ifndef DRACO_POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H
#define DRACO_POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H

// ros
#include <sensor_msgs/PointCloud2.h>

// point_cloud_transport
#include "draco_point_cloud_transport/CompressedPointCloud2.h"


//! assigns header, width, ... from compressed to regular
void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target, const draco_point_cloud_transport::CompressedPointCloud2& source);

//! assigns header, width, ... from regular to compressed
void assign_description_of_PointCloud2(draco_point_cloud_transport::CompressedPointCloud2& target, const sensor_msgs::PointCloud2& source);

//! assigns header, width, ... from compressedConstPtr to regular
void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target, const draco_point_cloud_transport::CompressedPointCloud2ConstPtr source);

#endif // DRACO_POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H
