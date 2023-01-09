#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "allan_ros2/allan_node.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<allan_ros::AllanNode>());
	rclcpp::shutdown();
	return 0;
}
