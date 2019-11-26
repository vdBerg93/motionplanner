#include "cloud_converter.hpp"

int main(int argc, char * * argv) {
	ros::init(argc, argv, "cloud_converter");

	ros::NodeHandle private_node_handle("~");

	robotics_practicals::CloudConverter cloud_converter(private_node_handle);

	ros::spin();

	return 0;
}
