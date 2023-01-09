#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rclcpp/serialization.hpp"
#include "allan_ros2/compute.hpp"
#include "allan_ros2/write.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "px4_msgs/msg/vehicle_imu_status.hpp"

namespace allan_ros {
    class AllanNode : public rclcpp::Node {
        public:
        AllanNode();
        ~AllanNode();

        private:
        void initialize_parameters();
        void configure_parameters();
        void deserialize_ros(rclcpp::SerializedMessage& serialized_msg);
        void deserialize_px4(rclcpp::SerializedMessage& serialized_msg);
        void process_bag();

        rclcpp::Parameter imu_topic;
        rclcpp::Parameter bag_path;
        rclcpp::Parameter publish_rate;
        rclcpp::Parameter sample_rate;
        rclcpp::Parameter msg_type;

        int skip_step;
        int sample_count;
        int sample_rate_;

        bool first_msg; 
        uint64_t current_timestamp; 
        uint64_t last_timestamp;

        rosbag2_cpp::StorageOptions bag_storage_options;
        rosbag2_cpp::ConverterOptions bag_converter_options;
        rosbag2_cpp::readers::SequentialReader bag_reader;
        rosbag2_storage::BagMetadata bag_info;
        std::chrono::seconds bag_length;

        using px4_imu = px4_msgs::msg::VehicleImuStatus;
        using ros_imu = sensor_msgs::msg::Imu;
    };
}