#ifndef CTRL_H
#define CTRL_H

#include "rrt/rrtplanner.h"
#include "rrt/vehicle.h"

using namespace std;

struct ControlCommand{
    double dc, ac;
    ControlCommand(double _dc, double _ac):dc(_dc),ac(_ac){};
};

class Controller{
    public:
        // Waypoint
        int IDwp; 
        geometry_msgs::Point Ppreview;
        // Control constants
        double dla_min, dla_vmin; 
        double Kp, Ki;
        int LAlong;
        // Control variables
        double tla, dla, ym;
        double E, iE;
        // Reference
        MyReference ref;
        int refIDend;
        // Vehicle parameters
        Vehicle veh;
        // Globals
        bool endreached;
        double Ts;

        Controller();
        void setReference(const MyReference& ref);
        ControlCommand getControls(const state_type& x);
    private:
        void updateLookahead(double v);
        void updateWaypoint(const state_type& x);
        double getAccelerationCommand(const state_type& x);
        double getSteerCommand(const state_type& x);
};

double getLateralError(const MyReference &ref, const state_type &x, const int& IDwp,const geometry_msgs::Point& Ppreview);
int findClosestPoint(const MyReference& ref, const geometry_msgs::Point& point, int ID);
void transformToVehicle(double (&xval)[3],double (&yval)[3],double (&Txval)[3],double (&Tyval)[3],const double (&x)[3]);
double interpolate(const double (&Txval)[3], const double (&Tyval)[3]);

#endif
