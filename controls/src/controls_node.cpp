// Global variables
using namespace std;
// Include STDLIB headers
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>
// Message headers
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// Headers from RRT package
#include <rrt/functions.h>
#include <rrt/datatypes.h>
#include <rrt/rrtplanner.h>
#include <rrt/vehicle.h>
// Control headers
#include "controls/datatypes.h"
#include "controls/msgmanager.h"
#include "controls/controller.h"
#include "controller.cpp"

int main( int argc, char** argv ){	
	// Initialize node
	ros::init(argc, argv, "controls_node");
	// Intialize object for node communication
	MsgManager msgManager;
	ros::Rate rate(25);
	// Initialize controller
	Controller ctrl;
	
	/*********************************************************************************
		The message manager is responsible for maintaining a motion plan queue.
	 	While the que is not empty, the controller repeatedly pick the first reference 
	 	and executes this untill the end is reached.
	*********************************************************************************/
	while(ros::ok()){
		if (msgManager.queueNotEmpty()){
			ctrl.setReference(msgManager.getFirstPlan());
			while(!ctrl.endreached){
				ControlCommand controls = ctrl.getControls(msgManager.getState());
				msgManager.publishControls(controls);
				ros::spinOnce();
			}
			msgManager.popFirstPlan();
		}else{
			ros::spinOnce();
		}
	}
}
