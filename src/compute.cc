#include <iostream>
#include "allan_ros2/compute.hpp"
#include <chrono>

using namespace imu;
using namespace std::chrono;

namespace compute {

    void allan_variance(const int& sample_rate) {
        
        if ((int)sample_buffer.size() <= 0) {
            return;
        }
        
        std::mutex mtx;

        #pragma omp parallel for
        for (int period = period_min; period < period_max; period++) {

            std::vector<std::vector<double>> averages;
            std::vector<double> current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            double period_time = period * 0.1; 
            int bag_size = period_time * sample_rate;

            for (int j = 0; j < ((int)sample_buffer.size() - bag_size); j += bag_size) {

                for (int m = 0; m < bag_size; m++) {
                
                    // compute mean - linear acceleration
                    current_average[0] += sample_buffer[j + m].linear_accel[0];
                    current_average[1] += sample_buffer[j + m].linear_accel[1];
                    current_average[2] += sample_buffer[j + m].linear_accel[2];

                    // compute mean - angular velocity   
                    current_average[3] += sample_buffer[j + m].angular_vel[0] * 180 / M_PI;
                    current_average[4] += sample_buffer[j + m].angular_vel[1] * 180 / M_PI;
                    current_average[5] += sample_buffer[j + m].angular_vel[2] * 180 / M_PI;
                }

                current_average[0] /= bag_size;
                current_average[1] /= bag_size;
                current_average[2] /= bag_size;
                current_average[3] /= bag_size;
                current_average[4] /= bag_size;
                current_average[5] /= bag_size;

                averages.push_back(current_average);
                current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }

            {
                std::lock_guard<std::mutex> lck(mtx);
                averages_map.insert({period, averages});
            }
        }

        for (int period = period_min; period < period_max; period++) {

            averages = averages_map.at(period);
            period_time = period * 0.1; 
            num_averages = averages.size();

            // compute variance 
            variance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            for (int k = 0; k < num_averages - 1; k++) {
                variance[0] += std::pow(averages[k + 1][0] - averages[k][0], 2);
                variance[1] += std::pow(averages[k + 1][1] - averages[k][1], 2);
                variance[2] += std::pow(averages[k + 1][2] - averages[k][2], 2);
                variance[3] += std::pow(averages[k + 1][3] - averages[k][3], 2);
                variance[4] += std::pow(averages[k + 1][4] - averages[k][4], 2);
                variance[5] += std::pow(averages[k + 1][5] - averages[k][5], 2);
            }

            avar = {
                variance[0] / (2 * (num_averages - 1)), variance[1] / (2 * (num_averages - 1)),
                variance[2] / (2 * (num_averages - 1)), variance[3] / (2 * (num_averages - 1)),
                variance[4] / (2 * (num_averages - 1)), variance[5] / (2 * (num_averages - 1))
            };
            
            // compute deviation 
            allan_deviation = {std::sqrt(avar[0]), std::sqrt(avar[1]), std::sqrt(avar[2]),
                                                std::sqrt(avar[3]), std::sqrt(avar[4]), std::sqrt(avar[5])};

            allan_write::allan_deviation(allan_deviation, period_time);
        }

    }
}
