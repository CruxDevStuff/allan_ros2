#pragma once 
#include <iostream>
#include <fstream>
#include <vector>
#include "allan_ros2/compute.hpp"
#include <iomanip>


namespace allan_write {
    struct AllanDeviationFormat {
        double period;
        double acc_x;
        double acc_y;
        double acc_z;
        double gyro_x;
        double gyro_y;
        double gyro_z; 
    };

    void allan_deviation(std::vector<double> variance, double period);
    inline std::ofstream file_writer("deviation.csv");
}