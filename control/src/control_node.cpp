using namespace std;

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <limits>
#include <cmath>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "car_msgs/Trajectory.h"
#include "car_msgs/MotionResponse.h"
#include "car_msgs/State.h"
// #include "car_msgs/ControlErrors.h"
#include "prius_msgs/Control.h"
#include "std_msgs/Float64.h"

#include "control/observer.h"

double convertQuaternionToEuler(geometry_msgs::Quaternion input);
double interpolate(const double (&Txval)[3], const double (&Tyval)[3]);
void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&x)[3]);
double getLateralError(const car_msgs::Trajectory path, const vector<double>& x, const int& IDwp);
double wrapTo2Pi(double x);
double wrapToPi(double x);

const bool debug = false;
const double pi = M_PI;

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
    // ros::Subscriber subMotion = nh.subscribe("/motionplanner/response",1000,&Observer::callbackMotion, &observer);
    ros::Subscriber subMotion = nh.subscribe("/path_publisher/path",1000,&Observer::callbackMotion, &observer);
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
    int bestID = std::numeric_limits<int>::max(); double bestDist = std::numeric_limits<double>::max();
    for(int i = 0; i!=path.x.size(); i++){
        double di = sqrt( pow(carState[0]-(path.x[i]),2) + pow(carState[1]-(path.y[i]),2));
        if (di<bestDist){
            bestDist = di;  bestID = i;
        }else{
            // break;
        }
    }

    // Get controls for best ID
    if (bestID==(path.x.size()-1)){
        // Brake when end of reference is reached
        a_cmd = -1;
        d_cmd = 0;
    }else if (bestID!=std::numeric_limits<int>::max()){
        // Longitudinal controller
        v_cmd = path.a_cmd[bestID];
        double v_error = v_cmd-carState[4];
        a_cmd = 0.5*v_error;
        a_cmd = std::min(std::max(double(-1),a_cmd),double(1));

        // Lateral error calculation
        double y_error = getLateralError(path, carState, bestID);

        // Path curvature calculation
        int IDmin;
        if (bestID==0){
            IDmin = bestID; 
        }else if (bestID==(path.x.size()-1)){
            IDmin = bestID-2;
        }else{
            IDmin = bestID-1;
        }
        // First order derivative
        double dy[2], dx[2], dydx[2];
        for(int i = 0; i!=2; i++){
            dx[i] = path.x[IDmin+i+1] - path.x[IDmin+i];
            dy[i] = path.y[IDmin+i+1] - path.y[IDmin+i];
            dydx[i] = dy[i]/dx[i];
            // cout<<"dydx["<<i<<"]="<<dydx[i]<<endl;
        }
            // Second order derivative
        double h2 = (dx[1]+dx[0])/2;
        double ddy = (dydx[1]-dydx[0])/h2;
        // cout<<"h2="<<h2<<", ddy="<<ddy<<endl;
        double slope = (dydx[1]-dydx[0])/2;
        // cout<<"slope="<<slope<<endl;
            // path curvature
        double kappa = ddy/pow( 1 + pow(slope,2),double(3)/double(2));

        // Heading error calculation
        double road_heading = atan2((dy[1]+dy[0])/2,(dx[1]+dx[0])/2);
        double jaw_error = wrapToPi(road_heading-carState[2]);

        // Path curvature controller
        double tla = 1;
        double xla = abs(carState[4])*tla;
        double veh_b = 1.61;
        double Kus = 0.0028;
        double dla = veh_b + xla;
        double delta = (veh_b + Kus*pow(carState[4],2))*(kappa + (2/pow(dla,2))*(y_error + xla*jaw_error));

        // Define steer command
        double dmax = 0.5435;
        delta = std::min(std::max(-dmax,delta),dmax);
        d_cmd = delta/dmax;

        if(debug){
            ROS_INFO_STREAM_THROTTLE(0.5,"path_x = ["<<path.x[IDmin]<<", "<<path.x[IDmin+1]<<", "<<path.x[IDmin+2]<<"]");
            ROS_INFO_STREAM_THROTTLE(0.5,"path_y = ["<<path.y[IDmin]<<", "<<path.y[IDmin+1]<<", "<<path.y[IDmin+2]<<"]");
            ROS_INFO_STREAM_THROTTLE(0.5,"error_y = "<<y_error<<"; error_jaw = "<<jaw_error<<"; curvature ="<<kappa);
            ROS_INFO_STREAM_THROTTLE(0.5,"dla = "<<dla<<"; d_cmd = "<<d_cmd<<"; a_cmd = "<<a_cmd<<"; v_ref = "<<v_cmd);
            ROS_INFO_STREAM_THROTTLE(0.5,"-----------------------------------");
        }

        assert( (-1<=d_cmd)&&(d_cmd<=1)&&"Steer command is out of limits! [-1,1]");
        assert( (-M_PI<=jaw_error)&&(M_PI>=jaw_error)&&"Heading error is out of limits! [0,2pi]");

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

void Observer::callbackMotion(const car_msgs::Trajectory& msg){
    path=msg;
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

double getLateralError(const car_msgs::Trajectory path, const vector<double>& x, const int& IDwp){
    // Determine the ID's of the reference that will be used for lateral error calculation
    int IDmin, IDmax;
    if (IDwp==0){
        IDmin = IDwp; IDmax = IDwp+2;
    }
    else if (IDwp==(path.x.size()-1)){
        IDmin = IDwp-2; IDmax = IDwp;
    }
    else{
        IDmin = IDwp-1; IDmax = IDwp+1;
    }
    // Extract points
    double xval[3] {path.x[IDmin],path.x[IDmin+1],path.x[IDmax]};
    double yval[3] {path.y[IDmin],path.y[IDmin+1],path.y[IDmax]};
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

double wrapTo2Pi(double x){
    // Wrap angle to [0,2pi]
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}

double wrapToPi(double x){
    x = fmod(x + pi,2*pi);
    if (x < 0)
        x += 2*pi;
    return x - pi;
}