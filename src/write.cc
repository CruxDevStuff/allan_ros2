#include <iostream>
#include "allan_ros2/write.hpp"

namespace write {
    void allan_deviation(std::vector<double> deviation, double period) {
        AllanDeviationFormat deviation_format;
        deviation_format.period = period; 
        deviation_format.acc_x = deviation[0];
        deviation_format.acc_y = deviation[1];
        deviation_format.acc_z = deviation[2];
        deviation_format.gyro_x = deviation[3];
        deviation_format.gyro_y = deviation[4];
        deviation_format.gyro_z = deviation[5];

        file_writer << std::setprecision(19) << period << std::setprecision(7) << "," << deviation_format.acc_x << "," << deviation_format.acc_y << "," << deviation_format.acc_z << ","
         << deviation_format.gyro_x << "," << deviation_format.gyro_y << "," << deviation_format.gyro_z << std::endl; 
    }
}