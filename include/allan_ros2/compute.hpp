#pragma once 
#include <Eigen/Core>
#include <mutex>
#include <map>
#include "allan_ros2/write.hpp"


namespace imu{
    struct SampleFormat {
        uint64_t time_stamp;
        Eigen::Vector3d linear_accel;
        Eigen::Vector3d angular_vel;
    };
    inline std::vector<SampleFormat> sample_buffer;
}

namespace compute{
    inline int period_min = 1;
    inline int period_max = 1000;
    inline double period_time;
    inline int num_averages;
    inline int bag_size;
    
    inline std::map<int,std::vector<std::vector<double>>> averages_map;
    inline std::vector<std::vector<double>> averages;
    inline std::vector<double> allan_deviation;
    inline std::vector<double> variance;
    inline std::vector<double> avar;
    inline std::vector<double> current_average;

    void allan_variance(const int& sample_rate);
}
