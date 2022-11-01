#include "pcl_div/pcl_div_node.hpp"
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

namespace pcl_div
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    PclNode::PclNode(const rclcpp::NodeOptions &node_options)
        : Node("pcl_div", node_options),
          pcl_gt_sub_(this, "~/input/pcl/gt", rclcpp::QoS{1}.best_effort().get_rmw_qos_profile()),
          pcl_pred_sub_(this, "~/input/pcl/pred", rclcpp::QoS{1}.best_effort().get_rmw_qos_profile()),
          sync_(SyncPolicy(100), pcl_gt_sub_, pcl_pred_sub_),
          reader_(std::make_unique<rosbag2_cpp::readers::SequentialReader>())
    {
        tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("morai_gt_1");

        rosbag2_storage::StorageOptions storage_options;
        rosbag2_cpp::ConverterOptions converter_options;

        storage_options.uri = std::string("/home/dominik/my-code/morai_sim_drive_rosbag2_220602_r1/morai_sim_drive_rosbag2_220602_r1_Normal");
        storage_options.storage_id = "sqlite3";
        converter_options.output_serialization_format = "cdr";
        reader_.open(storage_options, converter_options);
        while (reader_.has_next())
        {
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message = reader_.read_next();
            // std::cout << "Found topic name " << bag_message->topic_name << std::endl;
            std::vector<rosbag2_storage::TopicMetadata> topics = reader_.get_all_topics_and_types();

            if (bag_message->topic_name == "/lidar_front/points_xyzi") 
            {
                PointCloud2 extracted_test_msg;
                rclcpp::Serialization<PointCloud2> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(
                                                &extracted_serialized_msg, &extracted_test_msg);
                onPcl(extracted_test_msg);
            }
            if (bag_message->topic_name == "/perception/obstacle_segmentation/single_frame/pointcloud_raw") 
            {
                PointCloud2 extracted_test_msg;
                rclcpp::Serialization<PointCloud2> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(
                                                &extracted_serialized_msg, &extracted_test_msg);
                rclcpp::Time t = extracted_test_msg.header.stamp;

                writer_->write(extracted_test_msg, "/perception/obstacle_segmentation/single_frame/pointcloud_raw", t);
            }
            if (bag_message->topic_name == "/lidar_front/points_xyzi") 
            {
                PointCloud2 extracted_test_msg;
                rclcpp::Serialization<PointCloud2> serialization;
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(
                                                &extracted_serialized_msg, &extracted_test_msg);
                rclcpp::Time t = extracted_test_msg.header.stamp;

                writer_->write(extracted_test_msg, "/sensing/lidar/concatenated/pointcloud", t);
            }
        }
        // Gt ground and objects publishers
        pcl_gt_ground_ = this->create_publisher<PointCloud2>("~/pcl_gt_ground", rclcpp::SensorDataQoS());
        pcl_gt_obj_ = this->create_publisher<PointCloud2>("~/pcl_gt_obj", rclcpp::SensorDataQoS());

        // pcl_sub_ = this->create_subscription<PointCloud2>(
        //     "/perception/obstacle_segmentation/range_cropped/pointcloud",
        //     rclcpp::SensorDataQoS().keep_all(),
        //     std::bind(&PclNode::onPcl, this, _1));

        // sync_.registerCallback(std::bind(&PclNode::onPclSync, this, _1, _2));
    }

    void PclNode::onPcl(const PointCloud2 &msg)
    {
        RCLCPP_WARN(rclcpp::get_logger("pcl_div"), "START");
        PointCloud2 pcl_gt_ground, pcl_gt_obj;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *pcl_input);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_gt_ground_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_gt_obj_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_gt_ground_xyz->points.reserve(pcl_input->points.size());
        pcl_gt_obj_xyz->points.reserve(pcl_input->points.size());
        int cnt = 0;
        int cnt_all = 0;
        int cnt_no_0 = 0;
        std::unordered_map<int, int> a;
        for (const auto &point : pcl_input->points)
        {
            cnt_all++;
            if (point.x == 0 && point.y ==0 && point.z == 0)
            {
                cnt++;
            }
            // else if (point.intensity == 127 || point.intensity == 120 || point.intensity == 0 || point.intensity == 10)
            else if (point.intensity == 55 || point.intensity == 32 || point.intensity == 31 || (point.intensity >= 84 && point.intensity <= 90))
            {
                cnt_no_0++;
                pcl_gt_ground_xyz->points.push_back({point.x, point.y, point.z});
            }
            else
            {
                a[point.intensity] += 1;
                cnt_no_0++;
                pcl_gt_obj_xyz->points.push_back({point.x, point.y, point.z});
            }
        }
        RCLCPP_WARN(rclcpp::get_logger("pcl_div"), "all: %d, 0: %d, not 0: %d", cnt_all, cnt, cnt_no_0);
        
        for (auto& it : a) {
            RCLCPP_WARN(rclcpp::get_logger("pcl_div"), "Intensity: %d : %d", it.first, it.second);
        }

        pcl::toROSMsg(*pcl_gt_ground_xyz, pcl_gt_ground);
        pcl::toROSMsg(*pcl_gt_obj_xyz, pcl_gt_obj);
        pcl_gt_ground.set__header(msg.header);
        pcl_gt_obj.set__header(msg.header);

        rclcpp::Time t = msg.header.stamp;
        writer_->write(pcl_gt_ground, "pcl_div/pcl_gt_ground", t);
        writer_->write(pcl_gt_obj, "pcl_div/pcl_gt_obj", t);

        // std::cout << pcl_gt_ground.row_step << " + " << pcl_gt_obj.row_step << " = " << pcl_gt_ground.row_step + pcl_gt_obj.row_step << "    all: " << msg.row_step << std::endl;
        // pcl_gt_ground_->publish(pcl_gt_ground);
        // pcl_gt_obj_->publish(pcl_gt_obj);
    }

    void PclNode::onPclSync(const PointCloud2::ConstSharedPtr &msg_gt, const PointCloud2::ConstSharedPtr &msg_pred)
    {
        RCLCPP_WARN(rclcpp::get_logger("pcl_div"), "START SYNC");
        PointCloud2 pcl_gt_ground, pcl_gt_obj;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg_gt, *pcl_input);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_gt_ground_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_gt_obj_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_gt_ground_xyz->points.reserve(pcl_input->points.size());
        pcl_gt_obj_xyz->points.reserve(pcl_input->points.size());

        for (const auto &point : pcl_input->points)
        {
            if (point.intensity == 127 || point.intensity == 120 || point.intensity == 0 || point.intensity == 10)
            {
                pcl_gt_ground_xyz->points.push_back({point.x, point.y, point.z});
            }
            else
            {
                pcl_gt_obj_xyz->points.push_back({point.x, point.y, point.z});
            }
        }
        // rclcpp::Time t = msg_pred->header.stamp;
        pcl::toROSMsg(*pcl_gt_ground_xyz, pcl_gt_ground);
        pcl::toROSMsg(*pcl_gt_obj_xyz, pcl_gt_obj);
        pcl_gt_ground.set__header(msg_pred->header);
        pcl_gt_obj.set__header(msg_pred->header);
        // writer_->write(pcl_gt_ground, "pcl_div/pcl_gt_ground", t);
        // writer_->write(pcl_gt_obj, "pcl_div/pcl_gt_obj", t);
        // writer_->write(*msg_pred, "pcl_div/pcl_predicted_obj", t);

        pcl_gt_ground_->publish(pcl_gt_ground);
        pcl_gt_obj_->publish(pcl_gt_obj);
    }

} // namespace pcl_div

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_div::PclNode)
