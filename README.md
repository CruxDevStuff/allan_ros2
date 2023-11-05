# Allan ROS2 

ROS2 package for analysing noise parameters in IMU's using allan deviation. Designed for use with Kalibr and PX4. 

Note : This package is under development. If you find any bugs or errors create an issue, or even better a PR fixing it.

# Features 
1. Outputs noise parameters file for use with [Kalibr](https://github.com/ethz-asl/kalibr), Follows the [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
2. Supports both ```sensor_msgs/Imu.msg``` and ```px4_msgs/VehicleImuStatus.msg``` to read PX4-ROS2 bags, No conversion required.

# Usage 

This package has been tested with ROS2 Foxy and Humble

## Build
```bash 
   # create a new ros2 workspace 
   mkdir ~/allan_ws && cd ~/allan_ws
   mkdir src && cd src
   
   # clone 'px4_msgs' 
   git clone https://github.com/PX4/px4_msgs.git
   
   # clone the package
   git clone https://github.com/CruxDevStuff/allan_ros2
   
   cd ~/allan_ws
   
   # install dependencies
   rosdep install --from-paths src -y --ignore-src
   
   # build the workspace
   colcon build 
```
## Configure
Populate the parameters in ```~/allan_ws/src/allan_ros2/config/config.yaml``` with the appropriate values for you bag. [example configuration files](https://github.com/CruxDevStuff/allan_ros2_dev/tree/main/config)
```yaml
allan_node:
  ros__parameters:
     topic: # /your/topic 
     bag_path: # path/to/bag.db3
     msg_type: # use 'ros' for sensor_msgs::msg::Imu, use 'px4' for px4_msgs::msg::VehicleImuStatus
     publish_rate: # your imu publish rate (hz) 
     sample_rate: # rate to sample data from bag. Higher sample rates take longer to compute 
```
Note : Requires rebuilding after configuration ```colcon build --packages-select allan_ros2```. This will be fixed in a future release.
## Run 
Launch the node 
```bash
   # requires building after changing config
   colcon build --packages-select allan_ros2
   
   # source the workspace 
   source ~/allan_ws/install/setup.bash
   
   # launch the node 
   ros2 launch allan_ros2 allan_node.py
```
## Output 
The node outputs ```deviation.csv``` and contains the computed raw deviation values for all 6 axis. 

### Launch output
```
[allan_node-1] [INFO] [1673168790.110605813] [allan_node]: IMU Topic set to : # topic in config.yaml
[allan_node-1] [INFO] [1673168790.110669795] [allan_node]: Sample rate set to : # sample rate in config.yaml
[allan_node-1] [INFO] [1673168790.111149862] [rosbag2_storage]: Opened database # path set in config.yaml
[allan_node-1] [INFO] [1673168793.933100186] [allan_node]: Bag length (Seconds): # detected bag length 
[allan_node-1] [INFO] [1673168793.933151985] [allan_node]: Sampling data from bag...
[allan_node-1] [INFO] [1673168799.354297658] [allan_node]: Total samples : # samples taken from bag 
[allan_node-1] [INFO] [1673168799.354340691] [allan_node]: Computing variance and deviation
[allan_node-1] [INFO] [1673168804.424160542] [allan_node]: DONE, deviation output logged to 'deviation.csv'
```

### Example ```deviation.csv```

```
   # sample period     # accel x    # accel y    # accel z    # gyro x     # gyro y     # gyro z
78.10000000000000853, 0.0007594614, 0.003055848, 0.001219371, 0.001133672, 0.001355157, 0.001293785
78.20000000000000284, 0.0007684184, 0.003033959, 0.001216356, 0.001142654, 0.001362136, 0.001301119
78.30000000000001137, 0.0007739545, 0.002989617, 0.001177716, 0.001129241, 0.001408807, 0.001358467
78.40000000000000568, 0.0007518549, 0.003035895, 0.001149335, 0.00116584,  0.00140981,  0.001331341
```
### Noise parameters
To obtain the noise paramaters run ```analysis.py``` with deviation data. 
```analysis.py``` outputs ```imu.yaml``` and deviation plots. ```imu.yaml``` contains the ```random_walk``` and ```noise_density``` values for the IMU and follows the kalibr [IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) 
```bash
python3 src/allan_ros2/scripts/analysis.py --data deviation.csv
```
### Example ```imu.yaml```
```yaml
#Accelerometers
accelerometer_noise_density: 1.86e-03   # noise density (continuous-time)
accelerometer_random_walk:   4.33e-04   # bias random walk

#Gyroscopes
gyroscope_noise_density:     1.87e-04   # noise density (continuous-time)
gyroscope_random_walk:       2.66e-05   # bias random walk

rostopic:                               # fill your imu topic
update_rate:                            # fill your imu publish rate (hz)
```
### Plots
<img src="assets/gyro.png"/>
<img src="assets/acceleration.png"/>

# Acknowledgements 
Thanks to [Russell Buchanan](https://raabuchanan.com/). Computation code adapted from [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)
