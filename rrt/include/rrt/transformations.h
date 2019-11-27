#ifndef TRANSFORM_H
#define TRANSFORM_H

// Goal & state transformations
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs);
void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh);
// Tranformations from world to local
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs);
void transformStateWorldToCar(state_type& state, const state_type& carPose);							
void transformStateCarToRoad(state_type& states, const vector<double>& Cxy, const Vehicle& veh);	
void transformPathWorldToCar(vector<MyReference>& path, const vector<double>& carPose);
void transformPathCarToRoad(vector<MyReference>& path,const vector<double>& Cxy, const vector<double>& Cxs);
// Transform the received global car state to local
vector<double> transformStateToLocal(const vector<double>& worldState);
// Transformations from local to world
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose);
void transformStateRoadToCar(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformStateCarToRoad(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);							
// Path transformations
void transformPathWorldToCar(vector<Path>& path, const vector<double>& carPose);
void transformPathCarToRoad(vector<Path>& path,const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathRoadToCar(vector<Path>& path, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh);
void transformPathCarToWorld(vector<Path>& path, const vector<double>& worldState);

#endif