#ifndef RRT_H
#define RRT_H

// Configuration
using namespace std;
#include "rrt/vehicle.h"
#include "datatypes.h"

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
        MyRRT(vector<double> state, vector<double> _goalPose);
        //vector<double> sortNodes();
        void addNode(Node node);
        Node getNode(int ID);
        void addNodes(vector<Node> nodes);
        void getBestPath();
        vector<Node> tree;
    private: 
        
        
        
};

void initializeTree(MyRRT& RRT);
geometry_msgs::Point sampleAroundVehicle(vector<double> sampleBounds);
geometry_msgs::Point sampleOnLane(vector<double> lanes, double Lmax);
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub);
vector<int> sortNodesExplore(const MyRRT& rrt, const geometry_msgs::Point& sample);
vector<int> sortNodesOptimize(const MyRRT& rrt, const geometry_msgs::Point& sample);
bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample);
bool feasibleGoalBias(const MyRRT& rrt);
float dubinsDistance(geometry_msgs::Point S, Node N, int dir);
visualization_msgs::Marker createStateMsg(int ID, const vector<vector<double>> T);
visualization_msgs::Marker createReferenceMsg(int iD, const MyReference& ref);


// Reference generation functions
void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend);
MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir);

/* ----------------------------------------
        SIMPLE DATA OPERATIONS
-----------------------------------------*/
MyRRT::MyRRT(vector<double> state, vector<double> _goalPose){
	goalReached = 0;
	sortLimit = 10;
	direction = 1;
    goalPose = _goalPose;
    // Generate initial reference
	MyReference ref; 
    double xend = state[0]+cos(state[2])*ctrl_dla;
    double yend = state[1]+sin(state[2])*ctrl_dla;
    int N = floor( sqrt( pow(state[0]-xend,2) + pow(state[1]-yend,2) )/ref_res);
    ref.x = LinearSpacedVector(state[0],xend,N);
    ref.y = LinearSpacedVector(state[0],yend,N);
    for(int i = 0; i!=N; i++){
        ref.v.push_back(state[4]);
    }
    ref.dir = 1;
    // Initialize tree
    vector<state_type> T; T.push_back(state);
    Node initialNode(state,-1,ref,T,0,0,0);
	tree.push_back(initialNode);
}

// Add a node to the tree
void MyRRT::addNode(Node node){
	tree.push_back(node);
    if(debug_mode){
        cout<<"Added node."<<endl;
    }
}

// Add multiple nodes to the tree
void MyRRT::addNodes(vector<Node> nodes){
	for(int index = 0; index != nodes.size(); index++){
	};
}
#endif