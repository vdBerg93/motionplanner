#ifndef pubsubclass_h
#define pubsubclass_h

#include "rrt/rrtplanner.h"
#include "car_msgs/Reference.h"
#include <gazebo_msgs/LinkStates.h>
#include "prius_msgs/Control.h"

// #include "car_msgs/MotionPlan.h"
class MsgManager{
    public:
        MsgManager(){
            subPos = nh.subscribe("gazebo/link_states",50,&MsgManager::stateCallback,this);
            subMP  = nh.subscribe("mission_planner/motion_plan",50,&MsgManager::motionPlanCallback,this);
            pub    = nh.advertise<prius_msgs::Control>("prius/", 50, this);
        }
        void publishControls(const ControlCommand& controls){
            // Range 0 to 1, 1 is max throttle
            // Range 0 to 1, 1 is max brake
            // Range -1 to +1, +1 is maximum left turn
            // shift_gears: NO_COMMAND=0, NEUTRAL=1, FORWARD=2, REVERSE=3
            prius_msgs::Control msg;
            if (controls.ac>0){
                msg.throttle = (1/2)*controls.ac;
                msg.brake = 0;
            }else if(controls.ac<0){
                msg.throttle = 0;
                msg.brake = abs((1/6)*controls.ac);
            }else{
                msg.throttle = 0;
                msg.brake = 0;
            }
            msg.shift_gears = 2; // Shift to first gear
            msg.steer = (1/0.5435)*controls.dc;
            pub.publish(msg);
        }

        MyReference getFirstPlan(){
            return motionQueue.front();
        }
        void popFirstPlan(){
            motionQueue.erase(motionQueue.begin());
        }
        bool queueNotEmpty(){
            return (motionQueue.size()>0);
        }
        state_type getState(){
            return vehState;
        }
    private:
        ros::NodeHandle nh; 
        ros::Subscriber subPos;
        ros::Subscriber subMP;
        ros::Publisher pub;

        vector<MyReference> motionQueue;
        // car_msgs::MotionPlan motionPlan;
        state_type vehState;
        void stateCallback(gazebo_msgs::LinkStates msg){
            // Extract rear axle position in Quaternion
            int id = 1;  Quaternion Q;
            Q.w = msg.pose[id].orientation.w;
            Q.x = msg.pose[id].orientation.x;
            Q.y = msg.pose[id].orientation.y;
            Q.z = msg.pose[id].orientation.z;
            // Convert from quaternion to Euler angles;
            EulerAngles ang = ToEulerAngles(Q);
            // Update vehicle state in correct data type
            vehState[0]     = msg.pose[id].position.x;
            vehState[1]     = msg.pose[id].position.y;
            vehState[2]     = ang.yaw;
            // For debugging
            ROS_INFO_ONCE("State callback function is working!");
        };
        // void motionPlanCallback(car_msgs::MotionPlan msg){
        //     motionPlan = msg;
        //     ROS_INFO_ONCE("Motion plan callback function is working!");
        // };
        void motionPlanCallback(car_msgs::Reference msg){
            MyReference newRef;
            newRef.x = msg.x;
            newRef.y = msg.y;
            newRef.v = msg.v;
            newRef.dir = msg.dir;
            motionQueue.push_back(newRef);
            ROS_INFO("Added reference to motion queue.");
        };
};

#endif