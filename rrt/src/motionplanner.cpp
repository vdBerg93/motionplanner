// Function primitives
void publishPlan(vector<Node> &path, car_msgs::planmotion::Response &resp);
vector<double> getReqState(car_msgs::planmotion::Request req);
vector<double> getReqGoal(car_msgs::planmotion::Request req);

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
    private:
		
	public:
		vector<MyReference> motionplan; 	// Current motion plan in global coordinates
		vector<Node> tree;					// Tree
		ros::ServiceClient* clientPtr;		// Pointer to client
		vision_msgs::Detection2DArray det;	// 2D OBB
		bool planMotion(car_msgs::planmotion::Request &req, car_msgs::planmotion::Response &resp);
		bool updateObstacles();
};

bool MotionPlanner::updateObstacles(){
    car_msgs::getobstacles srv;
    (*clientPtr).call(srv);
	det = srv.response.obstacles;
}

bool MotionPlanner::planMotion(car_msgs::planmotion::Request &req, car_msgs::planmotion::Response &resp){
	cout<<"----------------------------------"<<endl;
	cout<<"Received request, processing..."<<endl;
	// Update global variables
	Vehicle veh; veh.setTalos();	
	updateLookahead(req.state[3]);	updateReferenceResolution(req.state[3]); 
	vmax = req.vmax; vgoal = req.goal[3];

	// Actual motion plan querie as in pseudocode
	vector<double> state = getReqState(req);	vector<double> goal = getReqGoal(req); updateObstacles();



	// Build the tree
	vector<Node> tree = buildTree(veh, ptrPub,state,goal);
	vector<Node> bestPath = extractBestPath(tree,1);

	publishPlan(bestPath, resp);
	cout<<"Replying to request..."<<endl;
	cout<<"----------------------------------"<<endl;
	return true;
}

void publishPlan(vector<Node> &path, car_msgs::planmotion::Response &resp){
	// Prepare message
	for(vector<Node>::iterator it = path.begin(); it!=path.end(); ++it){
		car_msgs::Reference ref; 	car_msgs::Trajectory tra;
		ref.dir = it->ref.dir;
		ref.x = it->ref.x;
		ref.y = it-> ref.y;
		ref.v = it->ref.v;
		resp.ref.push_back(ref);
		for(vector<vector<double>>::iterator it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
			tra.x.push_back( (*it2)[0]);
			tra.y.push_back( (*it2)[1]);
			tra.th.push_back( (*it2)[2]);
			tra.d.push_back( (*it2)[3]);
			tra.v.push_back( (*it2)[4]);
		}
		resp.tra.push_back(tra);
	}
	assert(resp.ref.size()==path.size());
}

// Get state data from service message
vector<double> getReqState(car_msgs::planmotion::Request req){
	vector<double> state;
	for(int i =0; i!=5; i++){
		state.push_back(req.state[i]);
	}
	return state;
}
// Get goal data from service message
vector<double> getReqGoal(car_msgs::planmotion::Request req){
	vector<double> goal;
	for(int i=0; i!=3; i++){
		goal.push_back(req.goal[i]);
	}
	return goal;
}

void project