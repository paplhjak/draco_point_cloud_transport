# \<DRACO POINT CLOUD TRANSPORT>

# Description
Plugin for ROS package point_cloud_transport, which uses Google Draco compression library for low-bandwidth transportation of PointCloud2 messages.

# Dynamic Reconfiguration

The plugin provides dynamic reconfiguration parameters, which can be used to change the compression during runtime. 

##Publisher
![publisher_settings](https://github.com/paplhjak/draco_point_cloud_transport/blob/master/readme_images/publisher.png)

### Encode and Decode Speed
By adjusting the **encode_speed** and **dedode_speed** parameters, one can adjust the speed at which the plugin encodes the point cloud. The higher the number, the higher the speed of encoding/decoding, but the worse the compression. For more detailed information, see documentation of [Draco](https://google.github.io/draco/).

### Encode Method
**Auto** method decides on the encoding method based on other parameter and should be used as the default method.

**KD-tree** method forces the encoder to use KD-tree encoding. If an attribute in the point cloud is of type float32, kd-tree encoding requires the attribute to be quantized.

**Sequential** method forces the encoder to use sequential encoding. Quantization can not be used with sequential encoding. Sequential encoding provides much worse compression than KD-tree, but is faster and keeps the arrangement of points in the point cloud intact. Therefore sequential encoding can be used to encode 2D point clouds such as from Kinect.

### Deduplicate
**Deduplicate** option tells the encoder whether or not to delete duplicate points in the point cloud, allowing for transport of smaller point clouds.

### Force Quantization
**Force_quantization** option forces the use of quantization and hence the KD-tree encoding method.

### Quantization of Attribute Types
**Quantization_POSITION**, **Quantization_NORMAL**, **Quantization_COLOR** etc. tells the encoder how many bits should be used for quantization of given attribute type. Attribute type of point cloud attribute is recognized based on a list of known names:
 - "x" - POSITION
 - "y" - POSITION
 - "z" - POSITION
 - "pos" - POSITION
 - "position" - POSITION
 - "rgb" - COLOR
 - "rgba" - COLOR
 - "r"  - COLOR
 - "g" - COLOR
 - "b" - COLOR
 - "a" - COLOR
 - "nx" - NORMAL
 - "ny" - NORMAL
 - "nz" - NORMAL
 - all others are encoded as GENERIC
 
 To specify custom quantization, one can either edit the list of recognized names or use **expert_quantization** and **expert_attribute_type** options.
 
### Expert Quantization

**Expert_quantization** option tell the encoder to use custom quantization values for point cloud attributes. Multiple POSITION attribute can therefore be encoded with varying quantization levels.

To set a quantization for a PointField entry "x" of point cloud which will be advertised on base topic *base_topic*, one must set the parameter:
/base_topic/draco/attribute_mapping/quantization_bits/x.

Example:
~~~~~~ bash
$ rosparam set /base_topic/draco/attribute_mapping/quantization_bits/x 16
~~~~~~

When using **expert_quantization**, user must specify the quantization bits for all PointField entries of point cloud.

### Expert Attribute Types

**Expert_attribute_types** option tell the encoder to use custom attribute types for encoding of point cloud attributes.

To set a type for a PointField entry "x" of point cloud which will be advertised on base topic *base_topic*, one must set the parameter:
/base_topic/draco/attribute_mapping/attribute_type/x.

Example:
~~~~~~ bash
$ rosparam set /base_topic/draco/attribute_mapping/attribute_type/x "'POSITION'"
~~~~~~

When using **expert_attribute_types**, user must specify the type for all PointField entries of point cloud. Accepted types are:
 - POSITION 
 - NORMAL
 - COLOR
 - TEX_COORD
 - GENERIC

When encoding rgb/rgba COLOR, user can specify to use the common rgba tweak of ROS (encoding rgba as 4 instances of 1 Byte instead of 1 instance of float32). To inform the encoder, that PointField entry "rgb" should be handled with the tweak, set parameter:

~~~~~~ bash
$ rosparam set /base_topic/draco/attribute_mapping/rgba_tweak/rgb true
~~~~~~

## Subscriber
![subscriber_settings](https://github.com/paplhjak/draco_point_cloud_transport/blob/master/readme_images/subscriber.png)

### Set Skip Dequantization of Attribute Types
**SkipDequantizationPOSITION**, **SkipDequantizationNORMAL**, **SkipDequantizationCOLOR** etc. tells the decoder to skip dequantization of given attribute types.
