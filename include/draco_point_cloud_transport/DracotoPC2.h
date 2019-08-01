#ifndef DRACO_POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
#define DRACO_POINT_CLOUD_TRANSPORT_DRACOTOPC2_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>

// draco
#include <draco/point_cloud/point_cloud.h>

// point_cloud_transport
#include "draco_point_cloud_transport/CompressedPointCloud2.h"
#include "draco_point_cloud_transport/conversion_utilities.h"

class DracotoPC2 {
public:
    //! Constructor.
    explicit DracotoPC2( std::unique_ptr<draco::PointCloud> && pc, const draco_point_cloud_transport::CompressedPointCloud2ConstPtr & compressed_PC2);

    //! Destructor
    //~DracotoPC2();

    //! Method for converting into sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 convert();

    //!
    std::vector<uint8_t> get_data();

        private:
    //! Message to be converted
    std::unique_ptr<draco::PointCloud> pc_;

    //! Structure to hold information about sensor_msgs::PointCloud2
    draco_point_cloud_transport::CompressedPointCloud2ConstPtr compressed_PC2_;

};


#endif // DRACO_POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
