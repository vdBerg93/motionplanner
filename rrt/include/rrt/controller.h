#ifndef CTRL_H
#define CTRL_H

#include "rrt/datatypes.h"

struct ControlCommand{
    double dc, ac;
    ControlCommand(double _dc, double _ac):dc(_dc),ac(_ac){};
};

class Controller{
    public:
        // Waypoint
        int IDwp; 
        geometry_msgs::Point Ppreview;
        // Lateral control
        double tla, dla_min, dla_vmin, dla_c, ym;
        // Longitudinal control
        double Kp, Ki, E, iE;
        // Globals
        bool endreached;

        Controller(const MyReference& ref, const state_type& x);
        ControlCommand getControls(const MyReference& ref, const Vehicle& veh, const state_type& x);
    private:
        double getAccelerationCommand(const Vehicle& veh, const MyReference& ref, const state_type& x);
        double getSteerCommand(const MyReference& ref, const state_type& x, const Vehicle& veh);
        void updateWaypoint(const MyReference& ref, const state_type& x);
};


double getLateralError(const MyReference &ref, const state_type &x, const int& IDwp,const geometry_msgs::Point& Ppreview);
int findClosestPoint(const MyReference& ref, const geometry_msgs::Point& point, int ID);
vector<point2D> transformToVehicle(vector<point2D> ref,vector<double> x);
double interpolate(vector<point2D> tref);
void updateLookahead(double v);

void updateReferenceResolution(double v);

#endif