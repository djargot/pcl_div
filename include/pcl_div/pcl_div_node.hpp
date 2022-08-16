#ifndef PCL_DIV__PCL_DIV_NODE_HPP_
#define PCL_DIV__PCL_DIV_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>
#include <vector>

namespace pcl_div
{
using sensor_msgs::msg::PointCloud2;

class PclNode : public rclcpp::Node
{
public:
    explicit PclNode(const rclcpp::NodeOptions &node_options);

    void onPcl(const PointCloud2 &msg);
    void onPclSync(const PointCloud2::ConstSharedPtr &msg_gt, const PointCloud2::ConstSharedPtr &msg_pred);

private:
    // ROS
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_gt_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_gt_obj_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    message_filters::Subscriber<PointCloud2> pcl_gt_sub_, pcl_pred_sub_;
    typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> SyncExact;
    SyncExact sync_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rosbag2_cpp::Reader reader_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
};

} // namespace pcl_div

#endif
