#pragma once
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace robotics_practicals {

// Function to convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
inline sensor_msgs::PointCloud2 toPointCloud2(sensor_msgs::PointCloud const & in) {
	sensor_msgs::PointCloud2 out;
	sensor_msgs::convertPointCloudToPointCloud2(in, out);
	return out;
}

// Function to convert sensor_msgs::PointCloud2 to pcl::PointCloud
inline pcl::PointCloud<pcl::PointXYZ> toPcl(sensor_msgs::PointCloud2 const & in) {
	pcl::PointCloud<pcl::PointXYZ> out;
	pcl::fromROSMsg(in, out);
	return out;
}

/// Function to convert sensor_msgs::PointCloud (deprecated type) to pcl::PointCloud via sensor_msgs::PointCloud2
inline pcl::PointCloud<pcl::PointXYZ> toPcl(sensor_msgs::PointCloud const & in) {
	return toPcl(toPointCloud2(in));
}

}
