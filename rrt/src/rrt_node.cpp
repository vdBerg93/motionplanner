// Global vars for debugging and plotting
bool draw_tree = 0;
bool draw_obs = 0;
bool draw_final_path = 0;
bool debug_mode = 0;
bool debug_reference = 0;
bool draw_states = 0;

// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vision_msgs/Detection2DArray.h>

// Global variables
double sim_dt;
double ctrl_tla, ctrl_dla, ctrl_mindla, ctrl_dlavmin, ctrl_Kp, ctrl_Ki;
double ref_res, ref_int, ref_mindist, vmax, vgoal;

// ros::Publisher* ptrPub;
// ros::ServiceClient* ptrSrv;

// Include header files
#include "rrt/functions.h"
#include "rrt/vehicle.h"
#include "rrt/rrtplanner.h"
#include "rrt/simulation.h"
#include "rrt/collision.h"
#include "rrt/controller.h"
#include "rrt/datatypes.h"
#include "car_msgs/Reference.h"
#include "car_msgs/planmotion.h"
#include "car_msgs/Trajectory.h"
#include "car_msgs/getobstacles.h"
 
// Include classes
#include "reference.cpp"
#include "rrtplanner.cpp"
#include "controller.cpp"
#include "simulation.cpp"
#include "collisioncheck.cpp"
#include "testers.cpp"

/* #############################
	CRITICAL TODO'S
##############################*/
// 1. Check cost function and node sorting heuristics

#include "car_msgs/MotionRequest.h"
#include "car_msgs/MotionReponse.h"
#include "car_msgs/State.h"
#include "motionplanner.cpp"
// #include "matlabgen/transformCarToRoad.h"
// #include "matlabgen/transformCarToRoad.cpp"

void updateParameters(){
	// Get parameters from server
	ros::param::get("ctrl/tla",ctrl_tla);
	ros::param::get("ctrl/mindla",ctrl_mindla);
	ros::param::get("ctrl/dlavmin",ctrl_dlavmin);
	ros::param::get("ctrl/refint",ref_int);
	ros::param::get("ctrl/refmindist",ref_mindist);
	ros::param::get("ctrl/sampleTime",sim_dt);
	ros::param::get("ctrl/Kp",ctrl_Kp);
	ros::param::get("ctrl/Ki",ctrl_Ki);
}

int main( int argc, char** argv ){	

	ROS_WARN_ONCE("In main: implement Car state straightening");

	
	// Initialize ros node handle
	ros::init(argc, argv, "rrt_node");
	ros::NodeHandle nh; ros::Rate rate(2);
	// Get parameters from server
	updateParameters();
	// Initialize motion planner object that handles services & callbacks
	MotionPlanner motionPlanner;
	// Create marker publisher for Rviz
	ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
	motionPlanner.pubPtr = &pubMarker; 	// Initialize global pointer to marker publisher
	
	// Register service with the master
	//ros::ServiceServer server = nh.advertiseService("planmotion", &MotionPlanner::planMotion,&motionPlanner);
	
	// Motion request subscriber
	ros::Subscriber sub  = nh.subscribe("motionrequest",100,&MotionPlanner::planMotion, &motionPlanner);

	// Motion response publisher
	ros::Publisher pubResp = nh.advertise<car_msgs::MotionReponse>("motionplan",100);
	motionPlanner.respPtr = &pubResp;

	ros::Subscriber subState = nh.subscribe("carstate",1,&MotionPlanner::updateState, &motionPlanner);

	// Register client for obstacle detector
	ros::ServiceClient client = nh.serviceClient<car_msgs::getobstacles>("getobstacles");
	motionPlanner.clientPtr = &client;

	// Give control to ROS
	cout<<"RRT Motion Planning Node running..."<<endl;
	while(ros::ok()){
		ros::spin();
	}
}

// // Testing new functions
// double Cxy[3] = {0.002,0.002,0.5};
// double Cxs[3] = {0.000392853825889,0.984741064715002,0.122939454240854};
// // double Csx[3] = {-0.000365738078703,1.013799256597537,-0.110077811210761};
// double X{95}, Y{16.5}; 
// double X0{95}, Y0{16.5}; 
// cout<<"X0="<<X0<<" Y0="<<Y0<<endl;
// transformCarToRoad(X,Y,Cxy,Cxs);
// cout<<"After straightening..."<<endl;
// cout<<"Xstraight="<<X<<" Ystraight="<<Y<<endl;
// transformRoadToCar(X,Y,Cxy,Cxs);
// cout<<"After bending..."<<endl;
// cout<<"Xend="<<X<<" Yend="<<Y<<endl;
// cout<<"Error="<<(sqrt( (X-X0)*(X-X0) + (Y-Y0)*(Y-Y0) ))<<endl;