// Vehicle controller
#include "rrt/simulation.h"
#include "rrt/controller.h"
#include "rrt/rrtplanner.h"
#include "rrt/vehicle.h"
#include "rrt/datatypes.h"
#include <boost/range/irange.hpp>
#include "controls/controller.h"


//*******************************
// CONTROLLER CLASS FUNCTIONS 
//*******************************
Controller::Controller(){
    ros::param::get("ctrl/tla",tla);
    ros::param::get("ctrl/mindla",dla_min);
    ros::param::get("ctrl/dlavmin",dla_vmin);
    ros::param::get("ctrl/sampleTime",Ts);
    ros::param::get("ctrl/Ki",Ki);
    ros::param::get("ctrl/Kp",Kp);
    LAlong = 2;    iE = 0;
    IDwp = 0; endreached = 0;
    veh.setTalos();
}

void Controller::setReference(const MyReference& _ref){
    IDwp = 0; endreached = 0; iE = 0;
    ref = _ref;
    refIDend = ref.v.size()-1;             // Index of last reference element
}

ControlCommand Controller::getControls(const state_type& x){
    updateLookahead(x[4]);  
    updateWaypoint(x);
    ControlCommand C {getSteerCommand(x),getAccelerationCommand(x)};
    return C;
}

void Controller::updateLookahead(double v){
	// double tla{1.5}, dla_min{3.2}, dla_vmin{3};
	double dla_c = dla_min - tla*dla_vmin; 
	dla = std::max(dla_min,dla_c+tla*std::abs(v));
};

void Controller::updateWaypoint(const state_type& x){
    // Use lookahead distance to update the preview point
    Ppreview.x = x[0] + dla*ref.dir*std::cos(x[2]);
    Ppreview.y = x[1] + dla*ref.dir*std::sin(x[2]);
    // Update the waypoint ID
    IDwp = findClosestPoint(ref, Ppreview, IDwp);

    if (IDwp==(ref.x.size()-1)){
        endreached = 1;
    };
};

double Controller::getAccelerationCommand(const state_type& x){
    int ID = std::min(IDwp+LAlong,refIDend);   // Make sure id does not exceed the vector length
    double E = ref.v[ID]-x[4];              // Error
    iE = iE + E*Ts;                     // Integral error
    // Calculate acceleration command and constrain it
    double aCmd = checkSaturation(veh.amin,veh.amax,Kp*E+Ki*iE);
    return aCmd;
};

double Controller::getSteerCommand(const state_type& x){
    ym = getLateralError(ref,x,IDwp,Ppreview);                      // Get the lateral error at preview point, perpendicular to vehicle 
    double cmdDelta = 2*((veh.L+veh.Kus*x[4]*x[4])/pow(dla,2))*ym; // Single preview point control (Schmeitz, 2017, "Towards a Generic Lateral Control Concept ...")
    return checkSaturation(-veh.dmax,veh.dmax,cmdDelta);;           // Constrain with actuator saturation limits
};

double getLateralError(const MyReference &ref, const state_type &x, const int& IDwp,const geometry_msgs::Point& Ppreview){
    // Determine the ID's of the reference that will be used for lateral error calculation
    int IDmin, IDmax;
    if (IDwp==0){
        IDmin = IDwp; IDmax = IDwp+2;
    }
    else if (IDwp==ref.x.size()){
        IDmin = IDwp-2; IDmax = IDwp;
    }
    else{
        IDmin = IDwp-1; IDmax = IDwp+1;
    }
    // Extract points
    double xval[3] {ref.x[IDmin],ref.x[IDmin+1],ref.x[IDmax]};
    double yval[3] {ref.y[IDmin],ref.y[IDmin+1],ref.y[IDmax]};
    // Extend preview point with vehicle heading
    double Xpreview[3] {Ppreview.x,Ppreview.y,x[2]};
    // Transform the extracted reference into local coordinates of the preview point
    double Txval[3], Tyval[3];
    transformToVehicle(xval,yval,Txval,Tyval,Xpreview);
    // Find the coordinate of local x-axis intersection to get the lateral error 
    double ym = interpolate(Txval,Tyval);
    return ym;
}

int findClosestPoint(const MyReference& ref, const geometry_msgs::Point& point, int ID){
    // Find the point along the reference that is closest to the preview point
    double dmin{inf}, di;
    int idmin = 0;
    for(int i = ID; i<ref.x.size(); i++){
        di = (ref.x[i]-point.x)*(ref.x[i]-point.x) + (ref.y[i]-point.y)*(ref.y[i]-point.y);
        // If next point is closer, update minimum
        if(di<dmin){ 
            dmin = di;
            idmin = i;
        }
        // else the points are increasing in distance (linear reference)
        else{
            return idmin;
        }
    }
    return idmin;
};

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
        Txval[i] = xval[i]*cos(x[2]) - x[0]*cos(x[2]) + yval[i]*sin(x[2])- x[1]*sin(x[2]);
        Tyval[i] = yval[i]*cos(x[2]) - x[1]*cos(x[2]) + xval[i]*sin(x[2])- x[0]*sin(x[2]);
    }
    // for(int i = 0; i<=2; i++){
    //     point2D point(ref[i].x*cos(x[2]) - x[0]*cos(x[2]) + ref[i].y*sin(x[2])- x[1]*sin(x[2]),
    //                   ref[i].y*cos(x[2]) - x[1]*cos(x[2]) + ref[i].x*sin(x[2])- x[0]*sin(x[2]));
    //     tref.push_back(point);
    // }
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
    // for(int i = 0; i<=2; i++){
    //     L = 1;
    //     for(int j = 0; j<=2; j++){
    //         if (i!=j){
    //             L = L*(tref[j].x)/(tref[i].x-tref[j].x);
    //         }
    //     }
    //     y = y + tref[i].y*L;
    // }
    return y;
}

