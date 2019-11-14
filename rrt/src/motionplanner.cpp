// Get updated obstacles from obstacle detection node
bool MotionPlanner::updateObstacles(){
    car_msgs::getobstacles srv;
    (*clientPtr).call(srv);
	det = srv.response.obstacles;
}

// State callback message
void MotionPlanner::updateState(car_msgs::State msg){
	state.clear();
	state.insert(state.begin(), msg.state.begin(), msg.state.end());
	assert(state.size()==6);
}

// Print a path to the terminal
void showPath(const vector<Path>& path){
	 for(auto it = path.begin(); it!=path.end(); it++){
		 cout<<"Refx = ["<<it->ref.x.front()<<", "<<it->ref.x.back()<<"]"<<endl;
		 cout<<"Refy = ["<<it->ref.y.front()<<", "<<it->ref.y.back()<<"]"<<endl;
		 cout<<"Trax = ["<<it->tra.front()[0]<<", "<<it->tra.back()[0]<<"]"<<endl;
		 cout<<"Tray = ["<<it->tra.front()[1]<<", "<<it->tra.back()[1]<<"]"<<endl;
	 }
}

// Print a node to the terminal
void showNode(const Node& node){
	cout<<"--- Node output ---"<<endl;
	cout<<"Refx = ["<<node.ref.x.front()<<", "<<node.ref.x.back()<<"]"<<endl;
	cout<<"Refy = ["<<node.ref.y.front()<<", "<<node.ref.y.back()<<"]"<<endl;
	cout<<"state= [";
	for(auto it = node.state.begin(); it!=node.state.end(); ++it){
		cout<<*it<<", ";
	}
	cout<<endl;
}

// Motion planner callback
void MotionPlanner::planMotion(car_msgs::MotionRequest req){
	fail_acclimit=0; fail_collision=0; fail_iterlimit=0; sim_count = 0;

	cout<<"----------------------------------"<<endl<<"Received request, processing..."<<endl;
	// Update variables
	Vehicle veh; veh.setTalos();	
	vector<double> worldState = state;
	vector<double> carPose = transformPoseWorldToCar(worldState);
	updateLookahead(carPose[4]);	updateReferenceResolution(carPose[4]); 
	vmax = req.vmax; vgoal = req.goal[3];
	updateObstacles();
	// Determine an upperbound of the lateral acceleration introduced by bending the path
	if (req.Cxy.size()>0){
		ay_road_max = abs(pow(worldState[4],2)*(2*req.Cxy[0]));
	}else{
		ay_road_max = 0;
	}
	
	// Get the array of committed motion plans
	transformPathWorldToCar(motionplan,worldState);
	// If road parametrization is available, convert motion spec to straightened scenario
	if(req.bend){
		transformPathCarToRoad(motionplan,req.Cxy,req.Cxs, veh);
		// Convert the obstacles
		for(auto it = det.begin(); it!=det.end(); ++it){
			transformPoseCarToRoad(it->obb.center.x,it->obb.center.y,it->obb.center.theta,req.Cxy,req.Cxs);
			ROS_WARN_STREAM("TODO: Implement velocity bending");
		}
		// Convert the goal
		transformPoseCarToRoad(req.goal[0],req.goal[1],req.goal[2],req.Cxy,req.Cxs);
		// transformStateCarToRoad(carPose,req.Cxy,veh);
	}

	// Predict state after tree build
	vector<double> Xend = carPose;

	// Initialize RRT planner
	MyRRT RRT(req.goal,req.laneShifts,req.Cxy, req.bend);	
	// cout<<"Created tree object"<<endl;
	double Tp = initializeTree(RRT, veh, motionplan, Xend);
	cout<<"Committed path time= "<<Tp<<endl;

	// Build the tree
	Timer timer(200); int iter = 0;				
	for(iter; timer.Get(); iter++){
		expandTree(veh, RRT, pubPtr, det, req.Cxy); 
	};
	cout<<"Expansion complete. Tree size is "<<RRT.tree.size()<<" after "<<iter<<" iterations"<<endl;

	// Select best path
	vector<Node> bestNodes = extractBestPath(RRT.tree,pubPtr);
	// Stop when tree is empty
	if(bestNodes.size()==0){		ROS_ERROR_STREAM("NO SOLUTION FOUND");	}
	// Select a part to commit here
	vector<Path> commit, plan;
	plan = convertNodesToPath(bestNodes);
	plan.insert(plan.begin(), motionplan.begin(), motionplan.end());


	if(Tp<Tcommit){
		commit = getCommittedPath(bestNodes, Tp);
	}else{
		ROS_INFO_STREAM("No commitment required.");
	}
	if(req.bend){
			transformPathRoadToCar(motionplan,req.Cxy,req.Cxs,veh);
			transformPathRoadToCar(commit,req.Cxy, req.Cxs,veh);
			transformPathRoadToCar(plan,req.Cxy,req.Cxs,veh);
	}
	transformPathCarToWorld(motionplan,worldState);
	transformPathCarToWorld(commit,worldState);
	transformPathCarToWorld(plan,worldState);

	if(commit.size()>0){
		publishPlan(plan); // Publish committed part and add to motion plan
	}
	
	publishPathToRviz(plan,pubPtr);	
	cout<<"Fail counters | col: "<<fail_collision<<" iter: "<<fail_iterlimit<<" acc: "<<fail_acclimit<<" sim it: "<<sim_count<<endl;
	cout<<"Replied to request..."<<endl<<"----------------------------------"<<endl;
}

// Message for clearing all markers
visualization_msgs::Marker clearMessage(){
	visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.ns = "motionplan";
    msg.action = visualization_msgs::Marker::DELETEALL;
}

// Message for publishing a path to Rviz
visualization_msgs::Marker generateMessage(const vector<Path>& path){
// Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "motionplan";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    msg.id = 0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.3;	// msg/LINE_LIST markers use only the x component of scale, for the line width

    msg.color.r = 0.0;
    msg.color.b = 0.0;
    msg.color.g = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration(5);
    
    geometry_msgs::Point p;
	for(auto it = path.begin(); it!=path.end(); ++it){
		for(auto it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
			p.x = (*it2)[0];
			p.y = (*it2)[1];
			p.z = 0;
			msg.points.push_back(p);
		}
	}
    return msg;    
}

// Publish a path to Rviz
void publishPathToRviz(const vector<Path>& path, ros::Publisher* ptrPub){
	visualization_msgs::Marker msg = generateMessage(path);
	visualization_msgs::MarkerArray msg2; msg2.markers.push_back(msg);
	ptrPub->publish(msg2);
}   

// Commit to a path section
vector<Path> getCommittedPath(vector<Node> bestPath, double& Tp){
	// Find committed reference
	vector<Path> commit;
	for(auto it = bestPath.begin(); it!=bestPath.end(); ++it){ 	// Loop through path
		Path path;		path.ref.dir = it->ref.dir;				// Initialize path
		for(int j = 1; j!=((*it).tra.size()); ++j){				// Start at second entry to avoid double values in path when merging sections
			Tp += sim_dt;										// Update committed time
			path.tra.push_back(it->tra[j]);						// Add state to committed path
			int IDwp = it->tra[j][7];						// Add waypoint for committed state
			path.ref.x.push_back(it->ref.x[IDwp]);				// push back waypoint
			path.ref.y.push_back(it->ref.y[IDwp]);				// push back waypoint	
			path.ref.v.push_back(it->ref.v[IDwp]);				// push back waypoint
			if (((Tp)>=(ctrl_tla + Tcommit))&&(path.ref.x.size()>=3)){		// Path should be at least three points long for controller to work
				commit.push_back(path);							
				return commit;
			}
		}
		commit.push_back(path);
	}
	return commit;	
}

// Future state prediction
void predictState(vector<double>& X0, const Vehicle& veh, double t){
	double dt = 0.01;
	for(int i = 0; i!=int(t/dt); i++){
		vector<double> dx = {X0[4]*cos(X0[2]), X0[4]*sin(X0[2]), (X0[4]/veh.L)*tan(X0[3])};
		for(int i = 0; i!=dx.size(); i++){
			X0[i] += dt*dx[i];
		}
	}
}

// Prepare motion response message
car_msgs::MotionResponse preparePathMessage(const vector<Path>& path){
	car_msgs::MotionResponse resp;
	for(auto it = path.begin(); it!=path.end(); ++it){
		// Prepare reference message
		car_msgs::Reference ref;
		ref.dir = it->ref.dir;
		ref.x.insert(ref.x.begin(), it->ref.x.begin(), it->ref.x.end());
		ref.y.insert(ref.y.begin(), it->ref.y.begin(), it->ref.y.end());
		ref.v.insert(ref.v.begin(), it->ref.v.begin(), it->ref.v.end());
		resp.ref.push_back(ref);
		// Prepare trajectory message
		car_msgs::Trajectory tra;
		for(int i = 0; i<it->tra.size(); i++){
			tra.x.push_back(it->tra[i][0]);
			tra.y.push_back(it->tra[i][1]);
			// tra.theta.push_back(it->tra[i][2]);
			// tra.delta.push_back(it->tra[i][3]);
			// tra.v.push_back(it->tra[i][4]);
			// tra.a.push_back(it->tra[i][5]);
			tra.a_cmd.push_back(it->tra[i][8]);
			tra.d_cmd.push_back(it->tra[i][9]);
		}
		resp.tra.push_back(tra);
	}
	return resp;
}

// Merge nodes into a path
vector<Path> convertNodesToPath(const vector<Node> &path){
	// Prepare message
	vector<Path> result;
	cout<<"pathsize="<<path.size()<<endl;
	for(auto it = path.begin(); it!=path.end(); ++it){
		Path segment;
		segment.ref = it->ref;
		segment.tra = it->tra;
		result.push_back(segment);
	}
	return result;
}

// Publish the motion plan
void MotionPlanner::publishPlan(const vector<Path>& plan){
		car_msgs::MotionResponse resp = preparePathMessage(plan);
		// Push into message
		if(commit_path){
			for(auto it = plan.begin(); it!=plan.end(); ++it){
				motionplan.push_back(*it);
			}
		}
	(*pubPlan).publish(resp);
}

// Publish the best path
void MotionPlanner::publishBestPath(const vector<Path>& path){
	car_msgs::MotionResponse resp = preparePathMessage(path);
	(*pubBest).publish(resp);
}

// 

/**************************************
 **** TRANSFORMATIONS OF 2D POINTS ****
 *************************************/

// Homogenous transformation from world to car
void transformPointWorldToCar(double& Xw, double& Yw, const vector<double>& carPose){
	double Xc = Xw*cos(carPose[2]) - carPose[0]*cos(carPose[2]) - carPose[1]*sin(carPose[2]) + Yw*sin(carPose[2]);
    double Yc = Yw*cos(carPose[2]) - carPose[1]*cos(carPose[2]) + carPose[0]*sin(carPose[2]) - Xw*sin(carPose[2]);
	Xw = Xc; Yw = Yc;
}

// Homogenous transformation from car to world
void transformPointCarToWorld(double& Xc, double& Yc, const vector<double>& carPose){
	double Xw = cos(carPose[2])*Xc - sin(carPose[2])*Yc + carPose[0];
	double Yw = sin(carPose[2])*Xc + cos(carPose[2])*Yc + carPose[1];
	Xc = Xw; Yc = Yw;
}

// Find closest point on the road centerline arc
vector<double> findClosestPointOnArc(const double& Xcar, const double& Ycar, const vector<double>& Cxy){
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
	double Xarc = ( pow(t24,1.0/3.0) *0.2403749283845681)/t2-Cxy[1]/(Cxy[0]*2.0)+1.0/ pow(Cxy[0],2)*t2*t17*1.0/ pow(t24,1.0/3.0)*0.3466806371753173;
    // y = Cxy[1]*x^2 + Cxy[0]*x + c0
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];
	vector<double> result {Xarc,Yarc};
	return result;
}

// Transform a point from curved-frame to straight-frame
void transformPointCarToRoad(double& Xcar, double& Ycar,const vector<double>& Cxy, const vector<double>& Cxs){
	vector<double> Parc = findClosestPointOnArc(Xcar,Ycar,Cxy);
	double Xarc{Parc[0]}, Yarc{Parc[1]};
    // ***** Use the previous point to calculate (S,rho) *****
    // Use continuous arc-length parametrization obtained by polynomial fitting
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));
    // half-plane test to determine sign
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    // if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
	if(Ycar<(Yarc - Xarc*(Cxy[1] + 2*Xarc*Cxy[0]) + Xcar*(Cxy[1] + 2*Xarc*Cxy[0]))){
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

// Transform a point from the curved-frame to straight-frame
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

/**************************************
 **** TRANSFORMATIONS OF STATES    ****
 *************************************/

void transformStateWorldToCar(state_type& state, const state_type& carPose){
	transformPointWorldToCar(state[0],state[1],carPose);
	state[2] -= carPose[2];
}
void transformStateCarToWorld(state_type& state, const state_type& carPose){
	transformPointCarToWorld(state[0],state[1],carPose);
	state[2] += carPose[2];
}

void transformStateCarToRoad(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	vector<double> Parc = findClosestPointOnArc(state[0],state[1],Cxy);
	double curvature = (2*Cxy[0])/ pow( ( pow(Cxy[1] + 2*Cxy[0]*Parc[0],2) + 1),(3/2));
	transformPoseCarToRoad(state[0],state[1],state[2],Cxy,Cxs);
	double delta = atan(curvature*veh.L); 						// Required steer angle to follow road curvature at x=0
	state[3] -= delta;											// Subtract steer angle to straighten states
}

void transformStateRoadToCar(state_type& state, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	transformPoseRoadToCar(state[0],state[1],state[2],Cxy,Cxs);
	vector<double> Parc = findClosestPointOnArc(state[0],state[1],Cxy);
	// Update steer angle	
	double curvature = (2*Cxy[0])/ pow( ( pow(Cxy[1] + 2*Cxy[0]*Parc[0],2) + 1),(3/2));
	double delta = atan(curvature*veh.L); 						// Required steer angle to follow road curvature at x=0
	state[3] += delta;											// Subtract steer angle to straighten states
}

/*******************************************************
 ****** POSE TRANSFORMATIONS ***************************
 ******************************************************/

vector<double> transformPoseWorldToCar(const vector<double>& worldState){
	vector<double> carPose = worldState;
	carPose[0] = 0; carPose[1] = 0; carPose[2]=0;
	return carPose;
}

void transformPoseCarToRoad(double& Xcar, double& Ycar, double& Hcar, const vector<double>& Cxy, const vector<double>& Cxs){
	// Transform obstacle to straightened road
	vector<double> Parc = findClosestPointOnArc(Xcar,Ycar,Cxy);
	double Xarc{Parc[0]}, Yarc{Parc[1]};
    // ***** Use the previous point to calculate (S,rho) *****
    double S = Cxs[0]*pow(Xarc,2) + Cxs[1]*Xarc + Cxs[2];
	double rho = sqrt(pow(Xarc-Xcar,2) + pow(Yarc-Ycar,2));

    // half-plane test to determine sign
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];
    if(Ycar<(dydx*Xcar + Yarc - dydx*Xarc)){
		rho = -rho;
	}
	double theta = atan2(Cxy[1], 1); 	// Heading of straightened road
	double Hroad = atan2(dydx,1);		// Heading of road at arc point
	double Hstraight = wrapTo2Pi( (Hcar-Hroad) + theta); // Calculate straightened heading

    // ***** Calculate (x,y)_straight *****
    // Rotate (S,rho) with slope, translate with C0
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];

	// ***** Update coordinates *****
	Xcar = Xstraight; Ycar = Ystraight; Hcar = Hstraight;
	assert((0<=Hcar)&(Hcar<=(2*pi)));
}

void transformPoseRoadToCar(double& Xstraight, double& Ystraight, double& Hstraight, const vector<double>& Cxy, const vector<double>& Cxs){
    //##### Find closest point on straightened road #####
	double Xroads = (Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight)/(pow(Cxy[1],2) + 1);
    double Yroads = Cxy[2] + (Cxy[1]*(Xstraight - Cxy[2]*Cxy[1] + Cxy[1]*Ystraight))/(pow(Cxy[1],2) + 1);
	//##### Get (S,rho) for straightened road ######
    double S = sqrt( pow(Xroads,2) + pow(Yroads-Cxy[2],2) );
    double rho = sqrt( pow(Xroads-Xstraight,2) + pow(Yroads-Ystraight,2) );
    // Do half-plane test to determine sign of rho
	if(Ystraight<(Cxy[1]*Xstraight + Cxy[2])){
        rho = -rho;
    }
    //##### Bend back to car coordinates #####
	double Xarc = -(Cxs[1] - sqrt(pow(Cxs[1],2) - 4*Cxs[0]*Cxs[2] + 4*Cxs[0]*S))/(2*Cxs[0]);	// Inverse of polyfit Cxs
    double Yarc = Cxy[0]*pow(Xarc,2) + Cxy[1]*Xarc + Cxy[2];									// Evaluate poly Cxy
    double dydx = 2*Cxy[0]*Xarc + Cxy[1];	// Slope of curved road
	double HroadC = atan2(dydx,1);			// Heading of curved road
	double HroadS = atan2(Cxy[1],1); 		// Heading of straight road
	double Hcar = wrapTo2Pi( (Hstraight - HroadS) + HroadC);
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

/*******************************************
 ***** TRANSFORMATION OF PATHS (T,Ref) *****
 ******************************************/
void transformPathRoadToCar(vector<Path>& path, const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	int iter{0};
	for(auto it = path.begin(); it!= path.end(); it++){
		// Transform the reference
		for(int i = 0; i != it->ref.x.size(); i++){
			transformPointRoadToCar(it->ref.x[i],it->ref.y[i],Cxy,Cxs);
		// Transform the trajectory
		}for(int i = 0; i != it->tra.size(); i++){
			transformStateRoadToCar(it->tra[i],Cxy,Cxs,veh);
		}
		iter++;
	}
}
void transformPathCarToWorld(vector<Path>& path, const vector<double>& worldState){
	for(auto it = path.begin(); it!= path.end(); it++){
		// Transform the reference
		for(int i = 0; i != it->ref.x.size(); i++){
			transformPointCarToWorld(it->ref.x[i],it->ref.y[i],worldState);
		// Transform the trajectory
		}for(int i = 0; i != it->tra.size(); i++){
			transformStateCarToWorld(it->tra[i],worldState);
		}
	}
}

void transformPathWorldToCar(vector<Path>& path, const vector<double>& carPose){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		// Transform the reference
		for(int i = 0; i!=(*itP).ref.x.size(); ++i){
			transformPointWorldToCar((*itP).ref.x[i], (*itP).ref.y[i], carPose);
		// Transform the state
		}for(int i = 0; i!=(*itP).tra.size(); ++i){
			transformStateWorldToCar((*itP).tra[i], carPose);
		}
	}
}

void transformPathCarToRoad(vector<Path>& path,const vector<double>& Cxy, const vector<double>& Cxs, const Vehicle& veh){
	for(auto itP = path.begin(); itP!=path.end(); itP++){
		// Transform the reference
		for(int i = 0; i!=(*itP).ref.x.size(); ++i){
			transformPointCarToRoad((*itP).ref.x[i], (*itP).ref.y[i],Cxy, Cxs);
		// Transform the state
		}for(int i = 0; i!=(*itP).tra.size(); ++i){
			transformStateCarToRoad((*itP).tra[i], Cxy, Cxs, veh);
		}
	}
}


