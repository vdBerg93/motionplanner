// Goal & state transformations
void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs);
void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh);
// Path input transformations
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose);
void transformPathWorldToCar(vector<MyReference>& path, const vector<double>& carState);
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPathCarToRoad(vector<MyReference>& path,const vector<double>& Cxy, const vector<double>& Cxs);
// Path output transformations
void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs);
void transformPathRoadToCar(vector<MyReference>& path, const vector<double>& Cxy, const vector<double>& Cxs);
void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose);
void transformPathCarToWorld(vector<MyReference>& path, const vector<double>& worldState);


// void transformPathRoadToCar(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);
void bendPlan(vector<MyReference>& plan, const vector<double>& Cxy, const vector<double>& Cxs);

// Perhaps unnessecary
void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs);
void bendTrajectory(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);
void bendPath(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs);


// void transformSegmentCarToWorld(MyReference& segment, const vector<double>& carState);
vector<MyReference> getCommittedPath(vector<Node> bestPath, double Tc);

// Motion planner object for handling services, callbacks & clients
struct MotionPlanner{
		vector<MyReference> motionplan; 		// Current motion plan in global coordinates
		vector<double> state;
		// RoadFrame roadFrame;
		ros::ServiceClient* clientPtr;			// Pointer to client
		ros::Publisher* pubPtr; 				// Pointer to Rviz markers
		ros::Publisher* pubPlan;
		ros::Publisher* respPtr;				// Pointer to response publisher
		vision_msgs::Detection2DArray det;		// 2D OBB
		// bool planMotion(car_msgs::planmotion::Request& req, car_msgs::planmotion::Response& resp);
		void planMotion(car_msgs::MotionRequest msg);
		bool updateObstacles();
		void updateState(car_msgs::State msg);
		void publishPlan(const vector<MyReference>& plan);
		void publishNodes(vector<Node> &path);
};

bool MotionPlanner::updateObstacles(){
    car_msgs::getobstacles srv;
    (*clientPtr).call(srv);
	det = srv.response.obstacles;
}

void MotionPlanner::updateState(car_msgs::State msg){
	state.clear();
	state.insert(state.begin(), msg.state.begin(), msg.state.end());
	assert(state.size()==6);
}

vector<double> convertCarState(const vector<double>& worldState){
	vector<double> carState = worldState;
	carState[0] = 0; carState[1] = 0; carState[2]=0;
	return carState;
}

 void showPath(const vector<MyReference>& path){
	 cout<<"---path---"<<endl;
	 for(auto it = path.begin(); it!=path.end(); it++){
		 for( int j = 0; j!=it->x.size(); ++j){
		 	cout<<"x= "<<it->x[j]<<" y= "<<it->y[j]<<endl;
		 }
	 }
 }

void MotionPlanner::planMotion(car_msgs::MotionRequest req){
	cout<<"----------------------------------"<<endl<<"Received request, processing..."<<endl;
	// Update variables
	Vehicle veh; veh.setTalos();	
	vector<double> worldState = state;
	vector<double> carState = convertCarState(worldState);
	updateLookahead(carState[4]);	updateReferenceResolution(carState[4]); 
	vmax = req.vmax; vgoal = req.goal[3];
	updateObstacles();

	// Get the array of committed motion plans
	// vector<MyReference> planCar = motionplan;
	// transformPathWorldToCar(planCar,worldState);
	transformPathWorldToCar(motionplan,worldState);
	// showPath(planCar);
	ROS_ERROR_STREAM("In MP: Check tranformation of motion plan");
	// If road parametrization is available, convert motion spec to straightened scenario
	if(req.bend){
		// transformPathCarToRoad(planCar,req.Cxy,req.Cxs);
		transformPathCarToRoad(motionplan,req.Cxy,req.Cxs);
		// showPath(planCar);
		// Convert the obstacles
		for(auto it = det.detections.begin(); it!=det.detections.end(); ++it){
			transformPoseCarToRoad(it->bbox.center.x,it->bbox.center.y,it->bbox.center.theta,req.Cxy,req.Cxy);
		}
		// Convert the goal
		transformPoseCarToRoad(req.goal[0],req.goal[1],req.goal[2],req.Cxy,req.Cxs);
		transformStates(carState,req.Cxy,veh);
	}
	// For debugging
	cout<<"Goal= "<<req.goal[0]<<", "<<req.goal[1]<<", "<<req.goal[2]<<", "<<req.goal[3]<<endl;
	cout<<"WState= "<<worldState[0]<<", "<<worldState[1]<<", "<<worldState[2]<<", "<<worldState[3]<<", "<<worldState[4]<<", "<<worldState[5]<<endl;
	cout<<"CState= "<<carState[0]<<", "<<carState[1]<<", "<<carState[2]<<", "<<carState[3]<<", "<<carState[4]<<", "<<carState[5]<<endl;
	
	// Initialize RRT planner
	MyRRT RRT(req.goal);	
	double Tc = initializeTree(RRT, veh, motionplan, carState); 
	cout<<"Initialized tree."<<endl;

	// Build the tree
	Timer timer(100); 				
	for(int iter = 0; timer.Get(); iter++){
		expandTree(veh, RRT, pubPtr, det); 
	};
	cout<<"Build complete, final tree size: "<<RRT.tree.size()<<endl;


	// Select best path
	vector<Node> bestPath = extractBestPath(RRT.tree,1);
	// Select a part to commit here
	
	if(Tc<Tcommit){
		vector<MyReference> commit = getCommittedPath(bestPath, Tc);
		if(req.bend){
			transformPathRoadToCar(motionplan,req.Cxy,req.Cxs);
			transformPathRoadToCar(commit,req.Cxy, req.Cxs);
			// showPath(commit);
		}
		transformPathCarToWorld(motionplan,worldState);
		transformPathCarToWorld(commit,worldState);
		// showPath(commit);
		// cout<<"finished bending..."<<endl;
		publishPlan(commit);
	}

	// FOR DEBUGGING ONLY
	if(req.bend){
		bendPath(bestPath,worldState,req.Cxy,req.Cxs);
		bendTrajectory(bestPath,worldState,req.Cxy,req.Cxs);
	}
	// Publish best path
	publishNodes(bestPath);
	cout<<"Replyed to request..."<<endl<<"----------------------------------"<<endl;
}

vector<MyReference> getCommittedPath(vector<Node> bestPath, double Tc){
	double Tnew{0}, i{0};
	// Find committed reference
	vector<MyReference> commit;
	for(auto it = bestPath.begin(); it!=bestPath.end(); ++it){
		double L = sqrt( pow(it->ref.x.back()-it->ref.x.front(),2) + pow(it->ref.y.back()-it->ref.y.front(),2) );
		double res = L/(it->ref.x.size()-1);
		MyReference path;
		path.dir = it->ref.dir;
		for(int j = 0; j!=((*it).ref.x.size()); ++j){
			Tnew += res/(it->ref.v[j]);;
			path.x.push_back(it->ref.x[j]);
			path.y.push_back(it->ref.y[j]);
			path.v.push_back(it->ref.v[j]);
			if (((Tc+Tnew)>=2*Tcommit)&&(path.x.size()>=3)){
				commit.push_back(path);
				cout<<"commited a ref of size: "<<path.x.size()<<endl;
				return commit;
			}
		}
		commit.push_back(path);
		cout<<"commited a ref of size: "<<path.x.size()<<endl;
	}
	return commit;	
}

void MotionPlanner::publishPlan(const vector<MyReference>& plan){
	car_msgs::MotionPlan resp;
	for(auto it = plan.begin(); it!=plan.end(); ++it){
		car_msgs::Reference ref;
		ref.dir = it->dir;
		ref.x.insert(ref.x.begin(), it->x.begin(), it->x.end());
		ref.y.insert(ref.y.begin(), it->y.begin(), it->y.end());
		ref.v.insert(ref.v.begin(), it->v.begin(), it->v.end());
		resp.refArray.push_back(ref);
		// motionplan.push_back(*it);
	}
	(*pubPlan).publish(resp);
}

void MotionPlanner::publishNodes(vector<Node> &path){
	car_msgs::MotionResponse resp;
	// Prepare message
	for(vector<Node>::iterator it = path.begin(); it!=path.end(); ++it){
		car_msgs::Reference ref; 
		ref.dir = it->ref.dir;
		ref.x = it->ref.x;
		ref.y = it-> ref.y;
		ref.v = it->ref.v;
		resp.ref.push_back(ref);
	}
	for(vector<Node>::iterator it = path.begin(); it!=path.end(); ++it){
		car_msgs::Trajectory tra;
		for(int i = 0; i!= it->tra.size(); i++){
			tra.x.push_back(it->tra[i][0]);
			tra.y.push_back(it->tra[i][1]);
			tra.theta.push_back(it->tra[i][2]);
			tra.delta.push_back(it->tra[i][3]);
			tra.v.push_back(it->tra[i][4]);
			tra.a.push_back(it->tra[i][5]);
		}
		resp.tra.push_back(tra);
	}
	(*respPtr).publish(resp);
	// respPtr->publish(resp)
	assert(resp.ref.size()==path.size());
}

/**************************************
 **** TRANSFORMATIONS *****************
 *************************************/

void transformStates(vector<double>& states, const vector<double>& Cxy, const Vehicle& veh){
	double curvature = (2*Cxy[0])/pow(pow(Cxy[1],2) + 1,3/2);	// Curvature at x=0;
	double delta = atan(curvature*veh.L); 						// Required steer angle to follow road curvature at x=0
	states[3] -= delta;											// Subtract steer angle to straighten states
	states[2] = (states[5]*veh.L)*tan(delta); 					// Update yawrate
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
}

void transformPointRoadToCar(double& Xstraight, double& Ystraight,const vector<double>& Cxy, const vector<double>& Cxs){
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
	Xcar = Xstraight; Ycar = Ystraight; Hcar = Hstraight;
	assert((0<=Hcar)&(Hcar<=(2*pi)));
}

void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs){
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


void bendPath(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs){
	for(auto it = path.begin(); it!= path.end(); it++){
		for(int i = 0; i != it->ref.x.size(); i++){
			transformPointRoadToCar(it->ref.x[i],it->ref.y[i],Cxy,Cxs);
			transformPointCarToWorld(it->ref.x[i], it->ref.y[i], worldState);
		}
	}
}
void transformPathRoadToCar(vector<MyReference>& path, const vector<double>& Cxy, const vector<double>& Cxs){
	for(auto it = path.begin(); it!= path.end(); it++){
		for(int i = 0; i != it->x.size(); i++){
			transformPointRoadToCar(it->x[i],it->y[i],Cxy,Cxs);
		}
	}
}
void transformPathCarToWorld(vector<MyReference>& path, const vector<double>& worldState){
	for(auto it = path.begin(); it!= path.end(); it++){
		for(int i = 0; i != it->x.size(); i++){
			transformPointCarToWorld(it->x[i],it->y[i],worldState);
		}
	}
}

void bendTrajectory(vector<Node>& path, const vector<double>& worldState, const vector<double>& Cxy, const vector<double>& Cxs){
	for(auto it = path.begin(); it!= path.end(); it++){
		for(auto it2 = it->tra.begin(); it2!= it->tra.end(); it2++){
			transformPoseRoadToCar((*it2)[0],(*it2)[1],(*it2)[2],Cxy,Cxs);
			transformPointCarToWorld((*it2)[0], (*it2)[1], worldState);
		}
	}
}

void transformPathWorldToCar(vector<MyReference>& path, const vector<double>& carState){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		for(int i = 0; i!=(*itP).x.size(); ++i){
			transformPointWorldToCar((*itP).x[i], (*itP).y[i], carState);
		}
	}
}

void transformPathCarToRoad(vector<MyReference>& path,const vector<double>& Cxy, const vector<double>& Cxs){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		for(int i = 0; i!=(*itP).x.size(); ++i){
			transformPointCarToRoad((*itP).x[i], (*itP).y[i],Cxy, Cxs);
		}
	}
}

void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	// Homogenous transformation from world to car
	double Xc = cos(carPose[2])*Xw + sin(carPose[2])*Yw - carPose[1]*sin(carPose[2]) - carPose[0]*cos(carPose[2]);
	double Yc = sin(carPose[2])*Xw + cos(carPose[2])*Yw - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}

void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose){
	// Homogenous transformation from car to world
	double Xw = cos(carPose[2])*Xc - sin(carPose[2])*Yc + carPose[0];
	double Yw = sin(carPose[2])*Xc + cos(carPose[2])*Yc + carPose[1];
	Xc = Xw; Yc = Yw;
}

