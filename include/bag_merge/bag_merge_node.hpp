#ifndef BAG_MERGE__BAG_MERGE_NODE_HPP_
#define BAG_MERGE__BAG_MERGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/synchronizer.h>

#include <memory>
#include <string>
#include <vector>

namespace bag_merge
{
using sensor_msgs::msg::PointCloud2;

class PclNode : public rclcpp::Node
{
public:
    explicit PclNode(const rclcpp::NodeOptions &node_options);

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rosbag2_cpp::Reader reader_1, reader_2;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
};

} // namespace bag_merge

#endif
