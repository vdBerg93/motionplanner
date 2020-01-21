#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cstdlib>
#include <cmath>


#include "car_msgs/State.h"
#include "car_msgs/LaneDet.h"
#include "road_perception_simulator/PolynomialRegression.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "road_perception_simulator/functions.h"

void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
vector<double> getCoefficients(const vector<double> carState, const double& theta_close, const double& resolution, const double& Radius);
visualization_msgs::MarkerArray generateMessage(const vector<double>& coef);
visualization_msgs::Marker createEmptyMsg();

using namespace std;
struct point2D{
    double x,y;
    point2D(double _x, double _y): x(_x), y(_y){}
};

struct RoadClass{
    double Radius, resolution;
    int N;
    vector<point2D> data;
    double x_close,y_close, theta_close;

    RoadClass(): Radius(50), resolution(0.01){
        N = round(2*pi/resolution);
        vector<double> theta = LinearSpacedVector(0,2*pi,N);
        for(auto it = theta.begin(); it!=theta.end(); it++){
            data.push_back(point2D(Radius*sin(*it), Radius*(1-cos(*it)) ));
        }
    }
    double getDist(const vector<double>& carState, point2D point){
        return sqrt( pow(carState[0]-point.x,2) + pow(carState[1]-point.y,2));
    }
    void updateClosestPoint(const vector<double> carState){
        double Dmin = inf;
        for(auto it = data.begin(); it!=data.end(); it++){
            double D = getDist(carState,*it);
            if (D<Dmin){
                Dmin = D;
                x_close = it->x; y_close = it->y;
            }
        }
        theta_close = asin(x_close/Radius);
    }
    vector<double> getCoefficients(const vector<double> carState){
    int N = round(M_PI_4/resolution);
    vector<double> angles = LinearSpacedVector(theta_close, theta_close+M_PI_4, N);
    vector<double> X, Y;
    // Generate road centerline in road coordinates
    for(auto it = angles.begin(); it!= angles.end(); it++){
        X.push_back(Radius*sin(*it)); Y.push_back(Radius*(1-cos(*it)));
        transformPointWorldToCar(X.back(), Y.back(),carState);
    }
    // Fit 2nd order polynomial
    vector<double> coefficients;
    PolynomialRegression <double> Poly;
    Poly.fitIt(X,Y,2,coefficients);
    return coefficients;
}
};



class MsgManager{
    public:
    MsgManager(ros::Publisher* _ptrPub, ros::Publisher* _ptrRviz, RoadClass* _ptrRoad): ptrPub(_ptrPub), ptrRviz(_ptrRviz), ptrRoad(_ptrRoad){
    }
    void stateCallback(const car_msgs::State& input){
        vector<double> carState = input.state;
        ptrRoad->updateClosestPoint( carState);
        // cout<<"Pclose="<<ptrRoad->x_close<<", "<<ptrRoad->y_close<<", "<<ptrRoad->theta_close<<endl;
        vector<double> coefficients = ptrRoad->getCoefficients(carState);
        ROS_INFO_STREAM("Got center coefficients: ["<<coefficients[0]<<", "<<coefficients[1]<<", "<<coefficients[2]<<"]");
        car_msgs::LaneDet output;
        output.coef = coefficients;
        ptrPub->publish(output);
        // Rviz publising
        visualization_msgs::MarkerArray msgRviz = generateMessage(coefficients);
        ptrRviz->publish(msgRviz);
    }
    private:
    ros::Publisher* ptrPub;
    ros::Publisher* ptrRviz;
    RoadClass* ptrRoad;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_perception_simulator_node");
	ros::NodeHandle nh;
    ros::Rate r(25);
    RoadClass road;
    // Make message manager & node communication
    ros::Publisher pubRoad = nh.advertise<car_msgs::LaneDet>("/centerline_coefficients",0);
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::MarkerArray>("centerline",1);
    MsgManager msgManager(&pubRoad, &pubMarker, &road);
    ros::Subscriber subState = nh.subscribe("/carstate",0,&MsgManager::stateCallback, &msgManager);

    ros::spin();
}

// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}


visualization_msgs::Marker createEmptyMsg(){
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "centerline";
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    return msg;    
}

// Message for publishing a path to Rviz (WORLD COORIDNATES)
visualization_msgs::MarkerArray generateMessage(const vector<double>& coef){
    assert(coef.size()>=3 && "Wrong coefficient size!");
    visualization_msgs::Marker msgClear = createEmptyMsg();
    visualization_msgs::MarkerArray rvizOut;
    
    // visualization_msgs::MarkerArray msgArray;
	// visualization_msgs::Marker msgClear = createEmptyMsg();
	// msgArray.markers.push_back(msgClear);
    
    // msgClear.
    rvizOut.markers.push_back(msgClear);
    vector<double> x = LinearSpacedVector(0,20,20);
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "centerline";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    msg.id = 1;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.5;

    msg.color.r = 1.0;
    msg.color.b = 1.0;
    msg.color.g = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration();
    geometry_msgs::Point p;
    for(auto it = x.begin(); it!=x.end(); it++){
        p.x = *it;
        p.y = coef[0] + coef[1]*(*it) + coef[2]*pow((*it),2);
        p.z = 0;
        msg.points.push_back(p);
    }
    rvizOut.markers.push_back(msg);
    return rvizOut;    
}