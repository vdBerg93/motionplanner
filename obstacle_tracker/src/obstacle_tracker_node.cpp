//#######################################################
//## Object detection for Prius Automated Vehicle #######
//#######################################################
#include <vector>
using namespace std;
bool DEBUG = 0;

void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
template<class T>
void rotateVector2D(T& X, T& Y, const T& angle){
	T Xnew =  cos(angle)*X + sin(angle)*Y;
	T Ynew = -sin(angle)*X + cos(angle)*Y;
	X = Xnew; Y = Ynew;
}

// Global variables
double Kalman_gain_pos{0.1};
double Kalman_gain_vel{0.01};
double Kalman_gain_meas{0.5};
double PCL_cluster_maxit{100};
double PCL_cluster_treshold{0.2};

// Include header files
#include <headers.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_converter/TestConfig.h>

#include <functions.h>
#include <kalman.cpp>
#include "observer.cpp"

void testKalman();

//#### MAIN FUNCTION ####################################
int main (int argc, char** argv)
{
	// testKalman();
  // std::setprecision(3);
  
	// Initialize ROS
	ros::init (argc, argv, "obstacle_tracker_node"); 	// Initialize ROS system
	ros::NodeHandle nh;				// Create nodehandle
	// car_msgs::getobstacles::Response test;
	Observer ObserveObject;
	for(int i = 0; i!=6; i++){
		ObserveObject.carState.push_back(0);
	}

	// Rviz publisher
	ros::Publisher pubR =nh.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 100);
	ObserveObject.pubRviz = &pubR;

	// Create service server
	ros::ServiceServer server = nh.advertiseService("getobstacles", &Observer::callbackService,&ObserveObject);

	// MPC publisher
	ros::Publisher pubMPC = nh.advertise<vision_msgs::Detection2DArray>("/detection_2D",100);
	ObserveObject.pubMPC = &pubMPC;

	// Car state subscriber
	ros::Subscriber sub = nh.subscribe("/carstate",1,&Observer::callbackState, &ObserveObject);


	// TF listener
	tf::TransformListener listener;
	ObserveObject.tfListener = &listener;

	// // Dynamic reconfiguration
	dynamic_reconfigure::Server<pcl_converter_node::TestConfig> parserver;
  	dynamic_reconfigure::Server<pcl_converter_node::TestConfig>::CallbackType f;
  	f = boost::bind(&callbackParameter, _1, _2);
  	parserver.setCallback(f);

	ros::Rate r(20);
	int startcount = 0;
	while( ros::ok() ){
		// Give TF some time to initialize
		if (startcount<20){
			startcount++;
		}else{
			geometry_msgs::Twist pedPos;
			tf::StampedTransform tfPed;
			geometry_msgs::Twist twistPed;
			// Lookup transformation of pedestrian link
    		ros::Time now = ros::Time::now();
    		listener.waitForTransform("base_link","ped_link_1", now, ros::Duration(3.0));
    		listener.lookupTransform("base_link", "ped_link_1", now, tfPed);
			listener.lookupTwist("base_link", "ped_link_1",now, ros::Duration(0.25), twistPed);
			// Update obstacle vector from detection
			car_msgs::Obstacle2D pedObs;
			pedObs.obb.center.x = tfPed.getOrigin().x();
			pedObs.obb.center.y = tfPed.getOrigin().y();
			pedObs.obb.center.theta = 0;
			pedObs.obb.size_x = 1.5; pedObs.obb.size_y = 1.5;
			// Calculate velocity w.r.t. car in world coordinates
			vector<double> state = ObserveObject.carState;
			// Include ego vehicle velocity
			pedObs.vel.linear.x = twistPed.linear.x + ObserveObject.carState[4];
			pedObs.vel.linear.y = twistPed.linear.y;
			// Pushback in vector
			ObserveObject.Obs.clear();
			ObserveObject.Obs.push_back(pedObs);
			// Send MPC message
			vision_msgs::Detection2DArray msg = generateMPCmessage(ObserveObject.Obs);
			ObserveObject.pubMPC->publish(msg);
			assert((ObserveObject.Obs.size()>0)&&"Wrong dimension of obstacles");
			ObserveObject.sendMarkerMsg(ObserveObject.Obs);
			// Update velocities
			ROS_INFO_STREAM("x="<<pedObs.obb.center.x<<", y="<<pedObs.obb.center.y<<", Vx="<<pedObs.vel.linear.x<<", Vy="<<pedObs.vel.linear.y);
			ROS_INFO_STREAM("Updated pedestrian detection");
		}
		ros::spinOnce();
		r.sleep();
	}
}


/*
			// ROS_INFO_STREAM("Pushed obstacle in vector...");
			// Send MPC message
			vision_msgs::Detection2DArray msg = generateMPCmessage(ObserveObject.Obs);
			ObserveObject.pubMPC->publish(msg);
			// ROS_INFO_STREAM("Sent the MPC message...");
			// Publish to Rviz
			ObserveObject.updateTrackersKF();
			// ROS_INFO_STREAM("Updated trackers...");
			ObserveObject.sendMarkerMsg(ObserveObject.Obs);
			// ROS_INFO_STREAM("Sent marker message...");
			ROS_INFO_STREAM("Updated obstacles and trackers.");

// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}

*/