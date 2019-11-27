//#######################################################
//## Object detection for Prius Automated Vehicle #######
//#######################################################
using namespace std;
bool DEBUG = 0;
// Include header files
#include <headers.h>
#include <functions.h>
#include <kalman.cpp>
#include "cloudConversion.cpp"



void testKalman();

//#### MAIN FUNCTION ####################################
int main (int argc, char** argv)
{
	// testKalman();
  // std::setprecision(3);
  
	// Initialize ROS
	ros::init (argc, argv, "pcl_converter_node"); 	// Initialize ROS system
	ros::NodeHandle nh;				// Create nodehandle
	// car_msgs::getobstacles::Response test;
	Observer ObserveObject;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/point_cloud", 1, &Observer::callbackPointcloud,&ObserveObject);

	ros::Subscriber subLane = nh.subscribe ("/lanedetection", 1, &Observer::callbackLane,&ObserveObject);

	// Rvis publisher
	ros::Publisher pubR =nh.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 100);
	ObserveObject.pubRviz = &pubR;

	// Create service server
	ros::ServiceServer server = nh.advertiseService("getobstacles", &Observer::callbackService,&ObserveObject);

	// TF listener
	tf::TransformListener listener;
	ObserveObject.tfListener = &listener;

	// Create a ROS publisher for the output point cloud
	// ros::Publisher pub = nh.advertise<car_msgs::getobstaclesResponse>("/pcl_converter_node/detections", 1);
	// ObserveObject.pubPtr = &pub;
	ros::Rate r(40);
	while( ros::ok() ){
		ros::spinOnce();
		r.sleep();
	}
}