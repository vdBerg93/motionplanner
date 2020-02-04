#include "cloud_converter.hpp"
#include "convert.hpp"

namespace robotics_practicals {

CloudConverter::CloudConverter(ros::NodeHandle & private_node_handle) : nh_(private_node_handle) {
	subscriber_ = nh_.subscribe("/prius/center_laser/scan", 1, &CloudConverter::point_cloud_callback, this);
	publisher_  = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1000);
}


void CloudConverter::point_cloud_callback(const sensor_msgs::PointCloud & in) {
	pcl::PointCloud<pcl::PointXYZ> cloud = toPcl(in);
	publisher_.publish(cloud);
}


}
