// Function primitives


// vector<double> getReqState(car_msgs::MotionRequest req);
vector<double> getReqGoal(const car_msgs::MotionRequest& req);
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs);
void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs);
void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh);

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
		vector<MyReference> motionplan; 	// Current motion plan in global coordinates
		vector<Node> tree;					// Tree
		vector<double> state;
		// RoadFrame roadFrame;
		ros::ServiceClient* clientPtr;		// Pointer to client
		ros::Publisher* pubPtr; 				// Pointer to Rviz markers
		ros::Publisher* respPtr;				// Pointer to response publisher
		vision_msgs::Detection2DArray det;	// 2D OBB
		// bool planMotion(car_msgs::planmotion::Request& req, car_msgs::planmotion::Response& resp);
		void planMotion(car_msgs::MotionRequest msg);
		bool updateObstacles();
		void updateState(car_msgs::State msg);
		void publishPlan(vector<Node> &path);
};

bool MotionPlanner::updateObstacles(){
    car_msgs::getobstacles srv;
    (*clientPtr).call(srv);
	det = srv.response.obstacles;
}

void MotionPlanner::updateState(car_msgs::State msg){
	state.clear();
	state.insert(state.begin(), msg.state.begin(), msg.state.begin()+5);
	assert(state.size()==6);
}

void MotionPlanner::planMotion(car_msgs::MotionRequest req){
	cout<<"----------------------------------"<<endl<<"Received request, processing..."<<endl;
	// Update variables
	Vehicle veh; veh.setTalos();	
	vector<double> worldState = state;
	vector<double> carState = worldState; carState[0]=0; carState[1]=0; carState[2]=0;
	updateLookahead(carState[3]);	updateReferenceResolution(carState[3]); 
	vmax = req.vmax; vgoal = req.goal[3];
	updateObstacles();
	cout<<"Goal"<<req.goal[0]<<", "<<req.goal[1]<<", "<<req.goal[2]<<", "<<req.goal[3]<<endl;
	// If road parametrization is available, convert motion spec to straightened scenario
	if(req.bend){
		// Convert the obstacles
		for(int i = 0; i!=det.detections.size(); i++){
			transformPoseCarToRoad(det.detections[i].bbox.center.x,det.detections[i].bbox.center.y,det.detections[i].bbox.center.theta,req.Cxy,req.Cxy);
		}
		// Convert the goal
		transformPoseCarToRoad(req.goal[0],req.goal[1],req.goal[2],req.Cxy,req.Cxs);
	}
	// Check bending
	for(int i = 0; i<=2; ++i){
		cout<<"Cxy: "<<req.Cxy[i]<<" Cxs: "<<req.Cxs[i]<<endl;
	}
	cout<<"Goal"<<req.goal[0]<<", "<<req.goal[1]<<", "<<req.goal[2]<<", "<<req.goal[3]<<endl;
	

	// Build the tree
	vector<Node> tree = buildTree(veh, pubPtr,carState,req.goal,det);
	vector<Node> bestPath = extractBestPath(tree,1);
	publishPlan(bestPath);
	sleep(5);
	cout<<"Replying to request..."<<endl<<"----------------------------------"<<endl;
}

void MotionPlanner::publishPlan(vector<Node> &path){
	car_msgs::MotionReponse resp;
	// Prepare message
	for(vector<Node>::iterator it = path.begin(); it!=path.end(); ++it){
		car_msgs::Reference ref; 	car_msgs::Trajectory tra;
		ref.dir = it->ref.dir;
		ref.x = it->ref.x;
		ref.y = it-> ref.y;
		ref.v = it->ref.v;
		resp.ref.push_back(ref);
	}
	(*respPtr).publish(resp);
	// respPtr->publish(resp)
	assert(resp.ref.size()==path.size());
}

// Get goal data from service message
vector<double> getReqGoal(const car_msgs::MotionRequest& req){
	vector<double> goal;
	goal.insert(goal.begin(), req.goal.begin(), req.goal.begin()+3);
	// for(int i=0; i!=3; i++){s
	assert(goal.size()==4);
	return goal;
}

void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh){
	double curvature = (2*Cxy[0])/pow(pow(Cxy[1],2) + 1,3/2);
	double delta = atan(curvature*veh.L);
	states[3] -= delta;
	states[4] = (states[5]*veh.L)*tan(delta);
}

void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs){
    // ***** Find closest point on the road centerline arc *****
    // x = arg min norm([Xcarar,Ycarar]-[xroad,yroad]) (solved symbolically in MATLAB)
	// Results are valid
	float t2 = abs(Cxy[0]);
	float t3 = pow(Cxy[0],3);
	float t4 = pow(Cxy[1],2);
	float t5 = Xcar*Cxy[0]*2.0;
	float t6 = Ycar*Cxy[0]*4.0;
	float t8 = Cxy[0]*Cxy[2]*4.0;
	float t11 = sqrt(3.0);
	float t7 = pow(t2,3);
	float t9 = 1.0/t3;
	float t10 = -t8;
	float t12 = Cxy[1]+t5;
	float t13 = pow(t12,2);
	float t14 = Cxy[1]*t7*9.0;
	float t15 = Xcar*Cxy[0]*t7*18;
	float t17 = t4+t6+t10-2.0;
	float t16 = t13*27;
	float t18 = pow(t17,3);
	float t19 = -t18;
	float t20 = t16+t19;
	float t21 = sqrt(t20);
	float t22 = t3*t11*t21;
	float t23 = t14+t15+t22;
	float t24 = t9*t23;
	float Xarc = ( pow(t24,1.0/3.0) *0.2403749283845681)/t2-Cxy[1]/(Cxy[0]*2.0)+1.0/ pow(Cxy[0],2)*t2*t17*1.0/ pow(t24,1.0/3.0)*0.3466806371753173;
    // y = c2*x^2 + c1*x + c0
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];

    // ***** Use the previous point to calculate (S,rho) *****
    // Use continuous arc-length parametrization obtained by polynomial fitting
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));
    // half-plane test to determine sign
    double dydx = 2*Cxy[1]*Yarc - Cxy[2];
    if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
		rho = -rho;
	}

    // ***** Calculate (x,y)_straight *****
    // Calculate heading of slope at (x=0)
    double theta = atan2(Cxy[1], 1);
    // Rotate (S,rho) with slope, translate with C0
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];
    // Pstraight = [cos(theta),-sin(theta);sin(theta),cos(theta)]*[S;rho] + [0;Cxy[2]];

	// ***** Update coordinates *****
	Xcar = Xstraight; Ycar = Ystraight;
	// ROS_INFO_STREAM("Xarc="<<Xarc<<" Yarc="<<Yarc);
	// ROS_INFO_STREAM("S="<<S<<" rho="<<rho);
	// ROS_INFO_STREAM("Xstraight="<<Xstraight<<" Ystraight="<<Ystraight);
}

void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs){
	ROS_WARN_ONCE("In tranformations: Extend with heading transformation");
    //##### Find point on straightened road #####
    double Xroads = (Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight)/(pow(Cxy[1],2) + 1);
    double Yroads = Cxy[2] + (Cxy[1]*(Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight))/(pow(Cxy[1],2) + 1);
    //##### Get (S,rho) ######
    double S = sqrt( pow(Xroads,2) + pow(Yroads-Cxy[2],2) );
    double rho = sqrt( pow(Xroads-Xstraight,2) + pow(Yroads-Ystraight,2) );
    if(Ystraight<(Cxy[1]*Xstraight + Cxy[2])){
        rho = -rho;
    }
    //##### Bend back to car coordinates #####
	double Xarc = -(Cxs[1] - sqrt(pow(Cxs[1],2) - 4*Cxs[0]*Cxs[2] + 4*Cxs[0]*S))/(2*Cxs[0]);
    // double Xarc = Csx[0]*pow(S,2) + Csx[1]*S + Csx[2];
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    // Calculate normal vector at point on arc
    double vx = 1;
    double vy = dydx;
    double L = sqrt ( pow(vx,2) + pow(vy,2) );
    double nx = -(1/L)*dydx;
    double ny = (1/L);
    // Calculate point in car coordinates
    double Xroadc = Xarc + nx*rho;
    double Yroadc = Yarc + ny*rho;
	// Update coordinates
	Xstraight = Xroadc; Ystraight = Yroadc;
}

void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs){
    // ***** Find closest point on the road centerline arc *****
    // x = arg min norm([Xcarar,Ycarar]-[xroad,yroad]) (solved symbolically in MATLAB)
	// Results are valid
	float t2 = abs(Cxy[0]);
	float t3 = pow(Cxy[0],3);
	float t4 = pow(Cxy[1],2);
	float t5 = Xcar*Cxy[0]*2.0;
	float t6 = Ycar*Cxy[0]*4.0;
	float t8 = Cxy[0]*Cxy[2]*4.0;
	float t11 = sqrt(3.0);
	float t7 = pow(t2,3);
	float t9 = 1.0/t3;
	float t10 = -t8;
	float t12 = Cxy[1]+t5;
	float t13 = pow(t12,2);
	float t14 = Cxy[1]*t7*9.0;
	float t15 = Xcar*Cxy[0]*t7*18;
	float t17 = t4+t6+t10-2.0;
	float t16 = t13*27;
	float t18 = pow(t17,3);
	float t19 = -t18;
	float t20 = t16+t19;
	float t21 = sqrt(t20);
	float t22 = t3*t11*t21;
	float t23 = t14+t15+t22;
	float t24 = t9*t23;
	float Xarc = ( pow(t24,1.0/3.0) *0.2403749283845681)/t2-Cxy[1]/(Cxy[0]*2.0)+1.0/ pow(Cxy[0],2)*t2*t17*1.0/ pow(t24,1.0/3.0)*0.3466806371753173;
    // y = c2*x^2 + c1*x + c0
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];

    // ***** Use the previous point to calculate (S,rho) *****
    // Use continuous arc-length parametrization obtained by polynomial fitting
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));
    // half-plane test to determine sign
    double dydx = 2*Cxy[1]*Yarc - Cxy[2];
    if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
		rho = -rho;
	}
	// 
	double Hstraight = wrapTo2Pi(atan2(dydx,1)-Hcar);

    // ***** Calculate (x,y)_straight *****
    // Calculate heading of slope at (x=0)
    double theta = atan2(Cxy[1], 1);
    // Rotate (S,rho) with slope, translate with C0
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];
    // Pstraight = [cos(theta),-sin(theta);sin(theta),cos(theta)]*[S;rho] + [0;Cxy[2]];

	// ***** Update coordinates *****
	ROS_WARN_ONCE("In transformPose: check is heading transformation are correct.");
	Xcar = Xstraight; Ycar = Ystraight; Hcar = Hstraight;
	assert((0<=Hcar)&(Hcar<=(2*pi)));
	// ROS_INFO_STREAM("Xarc="<<Xarc<<" Yarc="<<Yarc);
	// ROS_INFO_STREAM("S="<<S<<" rho="<<rho);
	// ROS_INFO_STREAM("Xstraight="<<Xstraight<<" Ystraight="<<Ystraight);
}

void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs){
	ROS_WARN_ONCE("In tranformations: Extend with heading transformation");
    //##### Find point on straightened road #####
    double Xroads = (Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight)/(pow(Cxy[1],2) + 1);
    double Yroads = Cxy[2] + (Cxy[1]*(Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight))/(pow(Cxy[1],2) + 1);
    //##### Get (S,rho) ######
    double S = sqrt( pow(Xroads,2) + pow(Yroads-Cxy[2],2) );
    double rho = sqrt( pow(Xroads-Xstraight,2) + pow(Yroads-Ystraight,2) );
    if(Ystraight<(Cxy[1]*Xstraight + Cxy[2])){
        rho = -rho;
    }
    //##### Bend back to car coordinates #####
	double Xarc = -(Cxs[1] - sqrt(pow(Cxs[1],2) - 4*Cxs[0]*Cxs[2] + 4*Cxs[0]*S))/(2*Cxs[0]);
    // double Xarc = Csx[0]*pow(S,2) + Csx[1]*S + Csx[2];
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
	double Hcar = wrapTo2Pi(atan2(dydx,1)+Hstraight);
    // Calculate normal vector at point on arc
    double vx = 1;
    double vy = dydx;
    double L = sqrt ( pow(vx,2) + pow(vy,2) );
    double nx = -(1/L)*dydx;
    double ny = (1/L);
    // Calculate point in car coordinates
    double Xroadc = Xarc + nx*rho;
    double Yroadc = Yarc + ny*rho;
	// Update coordinates
	Xstraight = Xroadc; Ystraight = Yroadc; Hstraight = Hcar;
}


/*
/////////////// MAIN WITH SERVICE /////////////////////
bool MotionPlanner::planMotion(car_msgs::planmotion::Request& req, car_msgs::planmotion::Response& resp){
	cout<<"----------------------------------"<<endl;
	cout<<"Received request, processing..."<<endl;
	// Update global variables
	cout<<"test0"<<endl;
	Vehicle veh; veh.setTalos();	
	cout<<"size state="<<req.state.size()<<endl;
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

// Get state data from service message
vector<double> getReqState(const car_msgs::MotionRequest& req){
	vector<double> state;
	for(int i =0; i!=5; i++){
		state.push_back(req.state[i]);
	}
	return state;
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
*/