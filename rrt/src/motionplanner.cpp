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

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
    vector<MyReference> motionplan; // Current motion plan in global coordinates
	vector<Node> tree;
	ros::ServiceClient* clientPtr;
    vision_msgs::Detection2DArray det;
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
	Vehicle veh; veh.setTalos();
	updateLookahead(req.state[3]);
	updateReferenceResolution(req.state[3]);
	vmax = req.vmax;	sim_dt = 0.1; 	vgoal = req.goal[3];
	// Extract state and goal from msg
	vector<double> state = getReqState(req);
	vector<double> goal = getReqGoal(req);
	// Build the tree
	vector<Node> tree = buildTree(veh, ptrPub,state,goal);
	vector<Node> bestPath = extractBestPath(tree,1);
	// Prepare message
	for(vector<Node>::iterator it = bestPath.begin(); it!=bestPath.end(); ++it){
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
	assert(resp.ref.size()==bestPath.size());
	cout<<"Replying to request..."<<endl;
	cout<<"----------------------------------"<<endl;
	return true;
}