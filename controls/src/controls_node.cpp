// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/MultiArrayDimension.h>


int main( int argc, char** argv ){	
	std::cout.precision(2);

	// Initialize ros node handle
	ros::init(argc, argv, "controls_node");
	ros::NodeHandle nh; ros::Rate rate(2);

	// Register localization subsriber
	// ros::Subscriber subPos = nh.subscribe();

	// Register motion speficication subsriber
	// ros::Subscriber subPos = nh.subscribe();
	
	// Register controls publisher
	// ros::Publisher pubCtrl = nh.advertise<...>("controls",10);

	while(ros::ok()){
		ros::spin();
	}
}
