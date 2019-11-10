#ifndef MP_H
#define MP_H

struct Path{
	MyReference ref;
	vector<state_type> tra;
};

void predictState(vector<double>& X0, const Vehicle& veh, double t);
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

// void transformPathRoadToCar(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);
// void bendPlan(vector<MyReference>& plan, const vector<double>& Cxy, const vector<double>& Cxs);

// Perhaps unnessecary
void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs);
// void bendTrajectory(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);
// void bendPath(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);
vector<Path> convertNodesToPath(const vector<Node> &path);

// void transformSegmentCarToWorld(MyReference& segment, const vector<double>& carPose);
vector<Path> getCommittedPath(vector<Node> bestPath, double& Tc);

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
		vector<Path> motionplan; 		// Current motion plan in global coordinates
		state_type state;
		// RoadFrame roadFrame;
		ros::ServiceClient* clientPtr;			// Pointer to client
		ros::Publisher* pubPtr; 				// Pointer to Rviz markers
		ros::Publisher* pubPlan;
		ros::Publisher* pubBest;				// Pointer to response publisher
		vector<car_msgs::Obstacle2D> det;		// 2D OBB
		// bool planMotion(car_msgs::planmotion::Request& req, car_msgs::planmotion::Response& resp);
		void planMotion(car_msgs::MotionRequest msg);
		bool updateObstacles();
		void updateState(car_msgs::State msg);
		void publishPlan(const vector<Path>& plan);
		void publishBestPath(const vector<Path>& path);
		MotionPlanner(){
			vector<double> emptystate = {0,0,0,0,0,0};
			state = emptystate;
		}
};

visualization_msgs::Marker clearMessage();
visualization_msgs::Marker generateMessage(const vector<Path>& path);
void publishPath(const vector<Path>& path, ros::Publisher* ptrPub);
#endif