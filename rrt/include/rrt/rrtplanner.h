#ifndef RRT_H
#define RRT_H

// Configuration
using namespace std;
#include "rrt/vehicle.h"
#include "datatypes.h"
#include "vision_msgs/Detection2DArray.h"

// Tree build timer with limit set in ms
struct Timer{
	clock_t tstart, tnow;
	double diff, timeLimit;
	Timer(double _timeLimit): tstart(clock()), timeLimit(_timeLimit){}
	bool Get(){
		tnow = clock();
		diff = diffclock(tnow,tstart);
		return 0 + (diff<timeLimit);
	}
	double diffclock(clock_t clock1, clock_t clock2){
		double diffticks = clock1 - clock2;
		double diffms = (diffticks)/(CLOCKS_PER_SEC/1000);
		return diffms;
	}
};

struct MyReference{
    vector<double> x;
    vector<double> y;
    vector<double> v;
    signed int dir;
};

struct Node{        
    vector<double> state;   // Node state
    int parentID;           // Parent ID
    vector<int> children;   // Children id's
    MyReference ref;        // Reference to reach node
    float costE;            // Costfunction for exploration
    float costS;            // Costfunction for selecting the best path in structured driving
    bool goalReached;       // Boolean stating whether goal has been reached
    vector<state_type> tra;
    Node(){};
    Node( vector<double> _state, int _parentID, MyReference _ref, vector<state_type> _tra, double _costE, double _costS, bool _goal) : state(_state), parentID(_parentID), ref(_ref), tra(_tra),costE(_costE), costS(_costS), goalReached(_goal){};
    void addChild(int child){children.push_back(child);}
};

class MyRRT{
    public:
        int sortLimit;
        bool reverseAllowed;
        bool goalReached;
        vector<double> goalPose;
        signed int direction;
        vector<double> laneShifts;   // Lane shifts. 1st element is goal lane. 2nd element is other lane
        vector<double> Cxy;

        // Tree iniitalization
        MyRRT(const vector<double>& _goalPose, const vector<double>& _laneShifts, const vector<double>& _Cxy);
        void addInitialNode(const vector<double>& state);
        // Tree operations
        void addNode(Node node);
        Node getNode(int ID);
        void addNodes(vector<Node> nodes);
        void getBestPath();
        vector<Node> tree;
    private: 
        
        
        
};

double initializeTree(MyRRT& RRT, const Vehicle& veh, vector<MyReference>& path, const vector<double>& carState);
geometry_msgs::Point sampleAroundVehicle(vector<double> sampleBounds);
geometry_msgs::Point sampleOnLane(const vector<double>& Cxy, vector<double> laneShifts, double Lmax);
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub, const vision_msgs::Detection2DArray& det, const vector<double>& Cxy);
vector<int> sortNodesExplore(const MyRRT& rrt, const geometry_msgs::Point& sample);
vector<int> sortNodesOptimize(const MyRRT& rrt, const geometry_msgs::Point& sample);
bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample);
bool feasibleGoalBias(const MyRRT& rrt);
float dubinsDistance(geometry_msgs::Point S, Node N, int dir);
visualization_msgs::Marker createStateMsg(int ID, const vector<vector<double>> T);
visualization_msgs::Marker createReferenceMsg(int iD, const MyReference& ref);


// Reference generation functions
void generateVelocityProfile(	MyReference& ref, const double& _v0, const int& IDwp, const double& vmax, const double& vend);
MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir);

/* ----------------------------------------
        SIMPLE DATA OPERATIONS
-----------------------------------------*/
// Add a node to the tree
void MyRRT::addNode(Node node){
	tree.push_back(node);
}

// Add multiple nodes to the tree
void MyRRT::addNodes(vector<Node> nodes){
	for(int index = 0; index != nodes.size(); index++){
	};
}

#endif
