#include <iostream>
#include <string>
#include <chrono>
#include "allan_ros2/allan_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace allan_ros {
    AllanNode::AllanNode():Node("allan_node") {
        initialize_parameters();
        configure_parameters();
        process_bag();
        RCLCPP_INFO(get_logger(), "Computing variance and deviation");
        sample_rate_ = sample_rate.as_int();
        compute::allan_variance(sample_rate_);
        RCLCPP_INFO(get_logger(), "DONE, deviation output logged to 'deviation.csv'");
    }

    AllanNode::~AllanNode() {}

    void AllanNode::initialize_parameters() {
        this->declare_parameter("topic", "/imu");
        this->declare_parameter("bag_path", "bag");
        this->declare_parameter("publish_rate", 10);
        this->declare_parameter("sample_rate", 10);
        this->declare_parameter("msg_type", "ros");
    }

    void AllanNode::configure_parameters() {
        imu_topic = get_parameter("topic");
        bag_path = get_parameter("bag_path");
        publish_rate = get_parameter("publish_rate");
        msg_type = get_parameter("msg_type");
        sample_rate = get_parameter("sample_rate");
        skip_step = int(publish_rate.as_int() / sample_rate.as_int());

        RCLCPP_INFO_STREAM_ONCE(get_logger(), "IMU Topic set to : " << imu_topic.as_string());
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "IMU Sample rate set to : " << sample_rate.as_int());
        // TODO : check all parameters are in the right format  
    }

    void AllanNode::deserialize_px4_vehicle_imu_status(rclcpp::SerializedMessage& serialized_msg) {
        rclcpp::Serialization<px4_vehicle_imu_status> serialization;
        px4_vehicle_imu_status deserialized_msg;

        serialization.deserialize_message(&serialized_msg, &deserialized_msg);

        imu::SampleFormat measurement;
        measurement.time_stamp = deserialized_msg.timestamp;
        measurement.linear_accel = Eigen::Vector3d(deserialized_msg.mean_accel[0],
                                                deserialized_msg.mean_accel[1], deserialized_msg.mean_accel[2]);

        measurement.angular_vel = Eigen::Vector3d(deserialized_msg.mean_gyro[0], 
                                                deserialized_msg.mean_gyro[1], deserialized_msg.mean_gyro[2]); 

        imu::sample_buffer.emplace_back(measurement);
    }

    void AllanNode::deserialize_px4_sensor_combined(rclcpp::SerializedMessage& serialized_msg) {
        rclcpp::Serialization<px4_sensor_combined> serialization;
        px4_sensor_combined deserialized_msg;

        serialization.deserialize_message(&serialized_msg, &deserialized_msg);

        imu::SampleFormat measurement;
        measurement.time_stamp = deserialized_msg.timestamp;
        measurement.linear_accel = Eigen::Vector3d(deserialized_msg.accelerometer_m_s2[0],
                                                deserialized_msg.accelerometer_m_s2[1], deserialized_msg.accelerometer_m_s2[2]);

        measurement.angular_vel = Eigen::Vector3d(deserialized_msg.gyro_rad[0], 
                                                deserialized_msg.gyro_rad[1], deserialized_msg.gyro_rad[2]); 

        imu::sample_buffer.emplace_back(measurement);
    }

    void AllanNode::deserialize_ros(rclcpp::SerializedMessage& serialized_msg) {
        rclcpp::Serialization<ros_imu> serialization;
        ros_imu deserialized_msg;
        serialization.deserialize_message(&serialized_msg, &deserialized_msg);

       imu::SampleFormat measurement;
        measurement.time_stamp = deserialized_msg.header.stamp.nanosec;
        measurement.linear_accel = Eigen::Vector3d(deserialized_msg.linear_acceleration.x,
                                                deserialized_msg.linear_acceleration.y, deserialized_msg.linear_acceleration.z);

        measurement.angular_vel = Eigen::Vector3d(deserialized_msg.angular_velocity.x, 
                                                deserialized_msg.angular_velocity.y, deserialized_msg.angular_velocity.z); 

        imu::sample_buffer.emplace_back(measurement);  
    }

    static bool ends_with(std::string_view str, std::string_view suffix) {
	    return str.size() >= suffix.size() && 0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix);
    }

    void AllanNode::process_bag() {
        bag_storage_options.uri = bag_path.as_string();

        if (ends_with(bag_path.as_string(), "mcap")) {
	        bag_storage_options.storage_id = "mcap";
	} else {
		bag_storage_options.storage_id = "sqlite3";
	}
        
        bag_converter_options.input_serialization_format = rmw_get_serialization_format();
        bag_converter_options.output_serialization_format = rmw_get_serialization_format();

        bag_reader.open(bag_storage_options, bag_converter_options);

        first_msg = true;

        // TODO : check if bag and topic exists, if not notify and exit 

        bag_info = bag_reader.get_metadata();
        bag_length = std::chrono::duration_cast<std::chrono::seconds>(bag_info.duration); 

        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Bag length (Seconds): " << bag_length.count());
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Sampling data from bag...");

        while (bag_reader.has_next()) {

            auto bag_message = bag_reader.read_next();

            if (bag_message->topic_name == imu_topic.as_string()) {
                
                sample_count += 1;

                if (sample_count % skip_step != 0 || sample_count / publish_rate.as_int() > bag_length.count()) {
                    continue;
                }

                if (first_msg) {
                    first_msg = false;
                    current_timestamp = bag_message->recv_timestamp;
                    last_timestamp = bag_message->recv_timestamp;
                }

                if (current_timestamp < last_timestamp) {
                    continue;
                }

                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

                // TODO : automatically detect message type using metadata

                if (msg_type.as_string() == "ros") {
                    deserialize_ros(serialized_msg);
                } else if (msg_type.as_string() == "px4_vehicleimustatus") {
                    deserialize_px4_vehicle_imu_status(serialized_msg);
                } else if (msg_type.as_string() == "px4_sensorcombined") {
                    deserialize_px4_sensor_combined(serialized_msg);
		        } else {
                    RCLCPP_ERROR(get_logger(), "Unknown message type, check 'config.yaml'");
                }
            }
        }

        auto buffer_length = (int)imu::sample_buffer.size();
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Total samples : " << buffer_length);
    }
} // namespace allan_ros
