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
#include "car_msgs/State.h"
#include "car_msgs/MotionResponse.h"

#include "functions.cpp"

int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "mission_planner_node");
	ros::NodeHandle nh;
    // Initialize communication class
    MsgManager msgManager;
    // Initialize message subscribers
    ros::Subscriber subState = nh.subscribe("/carstate",0,&MsgManager::stateCallback, &msgManager);
    ros::Subscriber subGoal = nh.subscribe("/move_base_simple/goal",1000,&MsgManager::goalCallback, &msgManager);
    // ros::Subscriber subMP = nh.subscribe("/motionplanner/response",0,&MsgManager::motionCallback, &msgManager);
    // Initialize message publishers
    ros::Publisher pubMP = nh.advertise<car_msgs::MotionRequest>("/motionplanner/request",0);
    msgManager.ptrPubMP = &pubMP;
    ros::Rate rate(5);
    // Give control to ROS for goal definition
    bool doReplanning;
    ros::param::get("/motionplanner/replan",doReplanning);
    
    while (ros::ok()){
        if (msgManager.goalReceived){
            msgManager.sendMotionRequest();
            // cout<<"Press any key to continue to next iteration"<<endl;
            // cin.get();
            // if (!doReplanning){
            //     sleep(10000);
            // }
        }else{
            ROS_INFO_STREAM_THROTTLE(1,"Waiting for goal pose from Rviz...");
        }

        ros::spinOnce();
        rate.sleep();
        
    }
}
