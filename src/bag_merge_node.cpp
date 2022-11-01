#include "bag_merge/bag_merge_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace bag_merge
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    PclNode::PclNode(const rclcpp::NodeOptions &node_options)
        : Node("bag_merge", node_options),
          reader_1(std::make_unique<rosbag2_cpp::readers::SequentialReader>()),
          reader_2(std::make_unique<rosbag2_cpp::readers::SequentialReader>())
    {
        tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("output_merged_ray");

        rosbag2_storage::StorageOptions storage_options_1;
        rosbag2_cpp::ConverterOptions converter_options_1;
        storage_options_1.uri = std::string("/home/dominik/my-code/ros/morai_gt_1");
        storage_options_1.storage_id = "sqlite3";
        converter_options_1.output_serialization_format = "cdr";
        reader_1.open(storage_options_1, converter_options_1);

        rosbag2_storage::StorageOptions storage_options_2;
        rosbag2_cpp::ConverterOptions converter_options_2;
        storage_options_2.uri = std::string("/home/dominik/dev/autoware/segmented_morai_slow_8_ray");
        storage_options_2.storage_id = "sqlite3";
        converter_options_2.output_serialization_format = "cdr";
        reader_2.open(storage_options_2, converter_options_2);

        while (reader_1.has_next())
        {
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message = reader_1.read_next();
            // std::vector<rosbag2_storage::TopicMetadata> topics = reader_.get_all_topics_and_types();
            RCLCPP_WARN(rclcpp::get_logger("pcl_div"), "START");

            if (bag_message->topic_name == "/sensing/lidar/concatenated/pointcloud") 
            {
                PointCloud2 extracted_test_msg;
                rclcpp::Serialization<PointCloud2> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(
                                                &extracted_serialized_msg, &extracted_test_msg);
                rclcpp::Time t = extracted_test_msg.header.stamp;

                writer_->write(extracted_test_msg, "/sensing/lidar/concatenated/pointcloud", t);

                std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message_2 = reader_2.read_next();
                PointCloud2 extracted_test_msg_2;
                rclcpp::Serialization<PointCloud2> serialization_2;
                rclcpp::SerializedMessage extracted_serialized_msg_2(*bag_message_2->serialized_data);
                serialization_2.deserialize_message(
                                                &extracted_serialized_msg_2, &extracted_test_msg_2);
                writer_->write(extracted_test_msg_2, "/perception/obstacle_segmentation/single_frame/pointcloud_raw", t);
            } else {
                PointCloud2 extracted_test_msg;
                rclcpp::Serialization<PointCloud2> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(
                                                &extracted_serialized_msg, &extracted_test_msg);
                rclcpp::Time t = extracted_test_msg.header.stamp;

                writer_->write(extracted_test_msg, bag_message->topic_name, t);
            }
        }
    }

} // namespace bag_merge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bag_merge::PclNode)

