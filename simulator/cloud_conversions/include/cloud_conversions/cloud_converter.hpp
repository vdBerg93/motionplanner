#pragma once
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

namespace robotics_practicals {

/// Class to subscribe to point cloud (old type) and publish point cloud 2 (supported type)
class CloudConverter {

public:

	/// Constructor receiving node handle from the main application and setting up subscriber to the point cloud
	CloudConverter(ros::NodeHandle & private_node_handle);

private:

	/// Ros Node handle as passed by the main application
	ros::NodeHandle nh_;

	/// subscriber to a PointCloud (old type)
	ros::Subscriber subscriber_;

	/// publisher of a PointCloud2
	ros::Publisher publisher_;

	/// Callback that received a point cloud from the ros topic server, converts it and publishes PointCloud2
	void point_cloud_callback(const sensor_msgs::PointCloud & in);

};


}
