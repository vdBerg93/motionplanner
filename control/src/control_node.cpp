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
#include "car_msgs/State.h"
// #include "car_msgs/ControlErrors.h"
#include "prius_msgs/Control.h"
#include "std_msgs/Float64.h"

#include "control/observer.h"

double convertQuaternionToEuler(geometry_msgs::Quaternion input);
double interpolate(const double (&Txval)[3], const double (&Tyval)[3]);
void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&x)[3]);
double getLateralError(const car_msgs::MotionResponse path, const vector<double>& x, const int& IDtr, const int& IDwp);

int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "control_node");
	ros::NodeHandle nh;
    ros::Rate r(25);
    // publisher init
    ros::Publisher pubControl = nh.advertise<prius_msgs::Control>("/prius",100);
    // Publish errors
    ros::Publisher pubError1 = nh.advertise<std_msgs::Float64>("/control/error_lateral",100);
    ros::Publisher pubError2 = nh.advertise<std_msgs::Float64>("/control/error_velocity",100);
    ros::Publisher pubError3 = nh.advertise<std_msgs::Float64>("/control/cmd_velocity",100);
    ros::Publisher pubError4 = nh.advertise<std_msgs::Float64>("/control/real_velocity",100);
    // Communication class init
    Observer observer(&pubControl, &pubError1, &pubError2, &pubError3, &pubError4);
    // Initialize message subscribers
    ros::Subscriber subState = nh.subscribe("/carstate",1000,&Observer::callbackState, &observer);
    ros::Subscriber subMotion = nh.subscribe("/motionplanner/response",1000,&Observer::callbackMotion, &observer);
    std::string userIn = "start";

    while (ros::ok()){
        // bool b1 = observer.publishControls();
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
            double di = sqrt( pow(carState[0]-(path.tra[it].x[i]),2) + pow(carState[1]-(path.tra[it].y[i]),2));
            if (di<bestDist){
                bestDist = di;  bestID = i;  bestIt = it;
            }else{
                // break;
            }
        }
    }
    
    // Get controls for best ID
    cout<<"best tra: "<<bestIt<<", bestid="<<bestID<<endl;
    if ((bestIt==(path.tra.size()-1))&&(bestID==(path.tra[bestIt].x.size()-1))){
        a_cmd = -1;
        d_cmd = 0;
    }else if (bestID!=std::numeric_limits<int>::max()){
        // Longitudinal controller
        // a_cmd = path.tra[bestIt].a_cmd[bestID];
        // v_cmd = path.tra[bestIt].v[bestID];
        v_cmd = path.tra[bestIt].a_cmd[bestID];
        double v_error = v_cmd-carState[4];
        a_cmd = 0.5*v_error;
        a_cmd = std::min(std::max(double(-1),a_cmd),double(1));

        // Lateral controller
        d_cmd = path.tra[bestIt].d_cmd[bestID];
        double y_error = getLateralError(path, carState, bestIt, bestID);
        // d_cmd = 0.5*y_error + 0.5*(d_cmd-carState[2]);
        // d_cmd = 2*y_error;
        d_cmd = std::min(std::max(double(-1),d_cmd),double(1));
        std_msgs::Float64 msg;
        msg.data = y_error;         ptrPubError1->publish(msg);
        msg.data = v_error;         ptrPubError2->publish(msg);
        msg.data = v_cmd;           ptrPubError3->publish(msg);
        msg.data = carState[4];     ptrPubError4->publish(msg);
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
        msg.throttle = a_cmd;
    }else if (a_cmd<0){
        msg.brake = abs(a_cmd);
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


void Observer::callbackState(const car_msgs::State& msg){
    carState=msg.state;
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

/***************************************
 ****      Controller itself     *******
 **************************************/

double getLateralError(const car_msgs::MotionResponse path, const vector<double>& x, const int& IDtr, const int& IDwp){
    // Determine the ID's of the reference that will be used for lateral error calculation
    int IDmin, IDmax;
    if (IDwp==0){
        IDmin = IDwp; IDmax = IDwp+2;
    }
    else if (IDwp==(path.tra[IDtr].x.size()-1)){
        IDmin = IDwp-2; IDmax = IDwp;
    }
    else{
        IDmin = IDwp-1; IDmax = IDwp+1;
    }
    // Extract points
    double xval[3] {path.tra[IDtr].x[IDmin],path.tra[IDtr].x[IDmin+1],path.tra[IDtr].x[IDmax]};
    double yval[3] {path.tra[IDtr].y[IDmin],path.tra[IDtr].y[IDmin+1],path.tra[IDtr].y[IDmax]};
    // Extend preview point with vehicle heading
    double Xpreview[3] {x[0],x[1],x[2]};
    // Transform the extracted reference into local coordinates of the preview point
    double Txval[3], Tyval[3];
    transformToVehicle(xval,yval,Txval,Tyval,Xpreview);
    // Find the coordinate of local x-axis intersection to get the lateral error 
    double ye = interpolate(Txval,Tyval);
    return ye;
}

void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&x)[3]){
    // Transform to vehicle coordinates of preview point with a homogenous transformation.
    //      H = [R,d;zeros(1,2),1];
    // 1. Define the rotation matrix and position vector
    //      R = [cos(X3),-sin(X3);sin(X3),cos(X3)];
    //      d = [X1;X2];
    // 2. Define the inverse of the homogenous transformation matrix
    //      Hinv = [R',-R'*d;zeros(1,2),1];
    // 3. Loop through the points and transform them
    //      pointTransformed = Hinv*[Rx;Ry;1];
    for(int i = 0; i<=2; i++){
        Txval[i] = xval[i]*cos(x[2]) - x[0]*cos(x[2]) - yval[i]*sin(x[2])+ x[1]*sin(x[2]);
        Tyval[i] = yval[i]*cos(x[2]) - x[1]*cos(x[2]) + xval[i]*sin(x[2])- x[0]*sin(x[2]);
        	// double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
            // double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
    }
    return;
}

double interpolate(const double (&Txval)[3], const double (&Tyval)[3]){
    // Do a second order Lagrange interpolation around three closest data points
    // The lateral error is equal to the y-coordinate of x-axis intersection
    double y{0}, L;
    for(int i = 0; i<=2; i++){
        L = 1;
        for(int j = 0; j<=2; j++){
            if (i!=j){
                L = L*(Txval[j])/(Txval[i]-Txval[j]);
            }
        }
        y = y + Tyval[i]*L;
    }
    return y;
}
