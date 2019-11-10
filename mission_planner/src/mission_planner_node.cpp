using namespace std;

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>

// Include messages for state and goal
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
// Include motion planner message
#include "car_msgs/MotionRequest.h"
#include "car_msgs/MotionResponse.h"

#include "functions.cpp"

int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "mission_planner_node");
	ros::NodeHandle nh;
    // Initialize communication class
    MsgManager msgManager;
    // Initialize message subscribers
    ros::Subscriber subState = nh.subscribe("/amcl_pose",1000,&MsgManager::stateCallback, &msgManager);
    ros::Subscriber subGoal = nh.subscribe("/move_base_simple/goal",1000,&MsgManager::goalCallback, &msgManager);
    ros::Subscriber subMP = nh.subscribe("/motionplanner/response",1000,&MsgManager::motionCallback, &msgManager);
    // Initialize message publishers
    ros::Publisher pubMP = nh.advertise<car_msgs::MotionRequest>("/motionplanner/request",100);
    msgManager.ptrPubMP = &pubMP;
    ros::Rate rate(10);
    // Give control to ROS for goal definition
    
    while (ros::ok()){
        if (msgManager.goalReceived){
            sendMotionRequest(msgManager.ptrPubMP, msgManager.goalC, msgManager.Vmax);
        }else{
            ROS_INFO_STREAM_THROTTLE(1,"Waiting for goal pose from Rviz...");
        }
        ros::spinOnce();
    }
}
