using namespace std;

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <limits>
#include <cmath>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "car_msgs/MotionResponse.h"
#include "prius_msgs/Control.h"

#include "control/observer.h"

double convertQuaternionToEuler(geometry_msgs::Quaternion input);

int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "control_node");
	ros::NodeHandle nh;
    ros::Rate r(25);
    // publisher init
    ros::Publisher pubControl = nh.advertise<prius_msgs::Control>("/prius",100);
    // Communication class init
    Observer observer(&pubControl);
    // Initialize message subscribers
    ros::Subscriber subState = nh.subscribe("/amcl_pose",1000,&Observer::callbackState, &observer);
    ros::Subscriber subMotion = nh.subscribe("/motionplanner/response",1000,&Observer::callbackMotion, &observer);
    std::string userIn = "start";

    while (ros::ok()){
        bool b1 = observer.publishControls();
        ros::spinOnce();
        // ROS_INFO_STREAM("Published controls.");
        r.sleep();
    }    
}

bool Observer::publishControls(){
    if (updateControls()){
        ptrPub->publish(genMoveMsg());
        return true;
    }else{
        // ROS_WARN_STREAM_THROTTLE(0.5,"IN CTRL: No motion plan available to perform");
        ptrPub->publish(genStaticMsg());
        return false;
    }
}

bool Observer::updateControls(){
    // Loop through the motion plan and find the waypoint
    int bestID = std::numeric_limits<int>::max(); double bestDist = std::numeric_limits<double>::max(); int bestIt = std::numeric_limits<int>::max();
    for(int it = 0; it!=path.tra.size(); it++){
        for(int i = 0; i!=path.tra[it].x.size(); i++){
            double di = sqrt( pow(carPose[0]-(path.tra[it].x[i]),2) + pow(carPose[1]-(path.tra[it].y[i]),2));
            if (di<bestDist){
                bestDist = di;  bestID = i;  bestIt = it;
            }else{
                break;
            }
        }
    }
    if ((bestIt==0)&&(bestID==0)){
        bestID++;
    }
    // Get controls for best ID
    cout<<"best tra: "<<bestIt<<", bestid="<<bestID<<endl;
    if (bestID!=std::numeric_limits<int>::max()){
        a_cmd = path.tra[bestIt].a_cmd[bestID];
        d_cmd = path.tra[bestIt].d_cmd[bestID];
        return true;
    }else{
        return false;
    }
}


prius_msgs::Control Observer::genMoveMsg(){
    double amax = 2;
    double amin = -2;
    double dmax = 0.54;
    prius_msgs::Control msg;
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = ' ';
    if (a_cmd>0){
        msg.brake = 0;
        msg.throttle = a_cmd/amax;
    }else if (a_cmd<0){
        msg.brake = a_cmd/amin;
        msg.throttle = 0;
    }else{
        msg.brake = 0;
        msg.throttle = 0;
    }
    // msg.brake = 0;      // [0,1]
    // msg.throttle = 0;   // [0,1]
    msg.steer = d_cmd/dmax;      // [-1,1]
    msg.FORWARD;
    return msg;
}

prius_msgs::Control Observer::genStaticMsg(){
    prius_msgs::Control msg;
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = ' ';
    msg.brake = 0.2;      // [0,1]
    msg.throttle = 0;   // [0,1]
    msg.steer = 0;      // [-1,1]
    msg.NEUTRAL;
    return msg;
}


void Observer::callbackState(const geometry_msgs::PoseWithCovarianceStamped& msg){
    double theta = convertQuaternionToEuler(msg.pose.pose.orientation);
    vector<double> CarPose{msg.pose.pose.position.x,msg.pose.pose.position.y,theta};
    carPose=CarPose;
    // ROS_INFO_STREAM("Updated vehicle state.");
    return;
}

void Observer::callbackMotion(const car_msgs::MotionResponse& msg){
    path=msg;
    ROS_INFO_STREAM("Updated motion plan.");
    return;
}

double convertQuaternionToEuler(geometry_msgs::Quaternion input){
    // Return the yaw angle from the input message
    double siny_cosp = 2 * (input.w * input.z + input.x * input.y);
    double cosy_cosp = 1 - 2 * (input.y * input.y + input.z * input.z);
    return atan2(siny_cosp, cosy_cosp);
}