
#include "draco_point_cloud_transport/conversion_utilities.h"

void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target, const draco_point_cloud_transport::CompressedPointCloud2& source)
{
    target.header = source.header;
    target.height = source.height;
    target.width = source.width;
    target.fields = source.fields;
    target.is_bigendian = source.is_bigendian;
    target.point_step = source.point_step;
    target.row_step = source.row_step;
    target.is_dense = source.is_dense;
}

void assign_description_of_PointCloud2(draco_point_cloud_transport::CompressedPointCloud2& target, const sensor_msgs::PointCloud2& source)
{
    target.header = source.header;
    target.height = source.height;
    target.width = source.width;
    target.fields = source.fields;
    target.is_bigendian = source.is_bigendian;
    target.point_step = source.point_step;
    target.row_step = source.row_step;
    target.is_dense = source.is_dense;;
}

void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target, const draco_point_cloud_transport::CompressedPointCloud2ConstPtr source)
{
    target.header = source->header;
    target.height = source->height;
    target.width = source->width;
    target.fields = source->fields;
    target.is_bigendian = source->is_bigendian;
    target.point_step = source->point_step;
    target.row_step = source->row_step;
    target.is_dense = source->is_dense;
}
