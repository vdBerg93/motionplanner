#pragma once
#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>

namespace robotics_practicals {

/// Class containing a node handle subscribing to 3D detections messages and publishes corresponding visualization markers
class Detection3dToMarkers {
public:

	/// Constructor initializing subscribers and publishers
	Detection3dToMarkers(ros::NodeHandle & nh);
private:

	/// Node handle to subscribe from topics and publish to topics
	ros::NodeHandle nh_;

	/// Subscriber for 3D detection array messages
	ros::Subscriber detections_sub_;

	/// Publisher for visualization marker array messages
	ros::Publisher markers_pub_;

	/// Callback function to execute when a new 3D detection array message is received
	void detections_callback(vision_msgs::Detection3DArray const & detections);
};

}
