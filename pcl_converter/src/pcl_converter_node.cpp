//#######################################################
//## Object detection for Prius Automated Vehicle #######
//#######################################################

// Include header files
#include <headers.h>
#include <car_msgs/getobstacles.h>

ros::Publisher pub;					// Create publisher


//#### Point cloud conversion function ########################
// 1. The message is converted to pcl format
// 2. The point cloud is filterd for the ground
// 3. The outliers are taken
// 4. Clusters are taken from the outliers.
// 5. The size and centerpoint from the clusters are taken
// 6. The size and coordinates are sent 
// #include <cloudConversion.cpp>
#include "cloudConversion.cpp"

//#### MAIN FUNCTION ####################################
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_converter_node"); 	// Initialize ROS system
	ros::NodeHandle nh;				// Create nodehandle
	// car_msgs::getobstacles::Response test;
	Observer ObserveObject;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/point_cloud", 1, &Observer::callbackPointcloud,&ObserveObject);
	// Create service server
	ros::ServiceServer server = nh.advertiseService("getobstacles", &Observer::callbackService,&ObserveObject);

	// Create a ROS publisher for the output point cloud
	// pub = nh.advertise<vision_msgs::Detection2DArray> ("/pcl_converter_node/detections", 1);

	// Spin
	ros::spin ();
}



