#include "detection_3d_to_markers.hpp"
#include <visualization_msgs/MarkerArray.h>

namespace robotics_practicals {

namespace {

/// Create a 3D visualization box with the detection msg
visualization_msgs::Marker toMarker(vision_msgs::Detection3D const & detection) {
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::ADD;
	marker.header = detection.header;
	marker.color.a = 0.5;
	marker.color.g = 1.0;
	marker.lifetime = ros::Duration(.2);
	marker.ns = "markers";
	marker.scale.x = 0.1;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.pose = detection.bbox.center;
	marker.scale.x = detection.bbox.size.x;
	marker.scale.y = detection.bbox.size.y;
	marker.scale.z = detection.bbox.size.z;
	return marker;
}

/// Create a marker array from detection array msg
visualization_msgs::MarkerArray toMarkerArray(vision_msgs::Detection3DArray const & detections) {
	visualization_msgs::MarkerArray result;
	if (detections.detections.empty()) { // Delete all markers if there are no detections
		visualization_msgs::Marker marker;
		marker.header = detections.header;
		marker.action == visualization_msgs::Marker::DELETEALL;
		result.markers.push_back(marker);
		return result;
	} else { // Add markers to array in case of detections
		for (std::size_t i = 0; i < detections.detections.size(); ++i) {
			visualization_msgs::Marker marker = toMarker(detections.detections.at(i));
			marker.id = i;
			result.markers.push_back(marker);
		}
		return result;
	}
}


}


Detection3dToMarkers::Detection3dToMarkers(ros::NodeHandle & nh) : nh_(nh) {
	ROS_INFO("Initialized 3d detection to markers node.");
	detections_sub_ = nh_.subscribe("/pcl_obstacle_detector_node/detections", 1, &Detection3dToMarkers::detections_callback, this);
	markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 100);
}

void Detection3dToMarkers::detections_callback(vision_msgs::Detection3DArray const & detections) {
	visualization_msgs::MarkerArray markers = toMarkerArray(detections);
	markers_pub_.publish(markers);
}

}
