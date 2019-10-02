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

ros::Publisher* ptrPub;
ros::ServiceClient* ptrSrv;

// Include header files
#include "rrt/functions.h"
#include "rrt/vehicle.h"
#include "rrt/rrtplanner.h"
#include "rrt/simulation.h"
#include "rrt/collision.h"
#include "rrt/controller.h"
#include "rrt/datatypes.h"
#include "car_msgs/Reference.h"
#include "car_msgs/State.h"
#include "car_msgs/planmotion.h"
#include "car_msgs/Trajectory.h"
 
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

vector<double> getReqState(car_msgs::planmotion::Request req){
	vector<double> state;
	for(int i =0; i!=5; i++){
		state.push_back(req.state[i]);
	}
	return state;
}
vector<double> getReqGoal(car_msgs::planmotion::Request req){
	vector<double> goal;
	for(int i=0; i!=3; i++){
		goal.push_back(req.goal[i]);
	}
	return goal;
}

bool planMotion(car_msgs::planmotion::Request &req, car_msgs::planmotion::Response &resp){
	cout<<"----------------------------------"<<endl;
	cout<<"Received request, processing..."<<endl;
	Vehicle veh; veh.setTalos();
	updateLookahead(req.state[3]);
	updateReferenceResolution(req.state[3]);
	vmax = req.vmax;	sim_dt = 0.1; 	vgoal = req.goal[3];
	// Extract state and goal from msg
	vector<double> state = getReqState(req);
	vector<double> goal = getReqGoal(req);
	// Build the tree
	vector<Node> tree = buildTree(veh, ptrPub,state,goal);
	vector<Node> bestPath = extractBestPath(tree,1);
	// Prepare message
	for(vector<Node>::iterator it = bestPath.begin(); it!=bestPath.end(); ++it){
		car_msgs::Reference ref; 	car_msgs::Trajectory tra;
		ref.dir = it->ref.dir;
		ref.x = it->ref.x;
		ref.y = it-> ref.y;
		ref.v = it->ref.v;
		resp.ref.push_back(ref);
		for(vector<vector<double>>::iterator it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
			tra.x.push_back( (*it2)[0]);
			tra.y.push_back( (*it2)[1]);
			tra.th.push_back( (*it2)[2]);
			tra.d.push_back( (*it2)[3]);
			tra.v.push_back( (*it2)[4]);
		}
		resp.tra.push_back(tra);
	}
	assert(resp.ref.size()==bestPath.size());
	cout<<"Replying to request..."<<endl;
	cout<<"----------------------------------"<<endl;
	return true;
}

int main( int argc, char** argv ){	
	std::cout.precision(2);
	
	// Initialize ros node handle
	ros::init(argc, argv, "rrt_node");
	ros::NodeHandle nh; ros::Rate rate(2);

	// Get parameters from server
	ros::param::get("ctrl/tla",ctrl_tla);
	ros::param::get("ctrl/mindla",ctrl_mindla);
	ros::param::get("ctrl/dlavmin",ctrl_dlavmin);
	ros::param::get("ctrl/refint",ref_int);
	ros::param::get("ctrl/refmindist",ref_mindist);
	ros::param::get("ctrl/sampleTime",sim_dt);
	ros::param::get("ctrl/Kp",ctrl_Kp);
	ros::param::get("ctrl/Ki",ctrl_Ki);

	// Subscribe to detection service
	//ros::ServiceClient client = nh.serviceClient<vision_msgs::Detection2DArray>("getdetections");
	//ptrSrv = &client;	// Initialize global pointer to client

	// Create marker publisher for Rviz
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
	ptrPub = &marker_pub; 	// Initialize global pointer to marker publisher

	// Register service with the master
	ros::ServiceServer server = nh.advertiseService("planmotion", &planMotion);

	while(ros::ok()){
		ros::spin();
	}
}

