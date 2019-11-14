#include <limits>
#include "rrt/collision.h"
#include "rrt/rrtplanner.h"
#include "rrt/simulation.h"

using namespace std;

MyRRT::MyRRT(const vector<double>& _goalPose, const vector<double>& _laneShifts, const vector<double>& _Cxy, const bool& _bend):
	bend(_bend), goalReached(0), sortLimit(10), direction(1), goalPose(_goalPose), laneShifts(_laneShifts), Cxy(_Cxy){}

void MyRRT::addInitialNode(const vector<double>& state){
	// Set the first node in the tree at the current preview point of the lateral controller
	MyReference ref; 
    double xend = state[0]+cos(state[2])*ctrl_dla; 								// Xpreview
    double yend = state[1]+sin(state[2])*ctrl_dla;								// Ypreview
    int N = floor( sqrt( pow(state[0]-xend,2) + pow(state[1]-yend,2) )/ref_res);// Size
    ref.x = LinearSpacedVector(state[0],xend,N);								// Reference (x)
    ref.y = LinearSpacedVector(state[0],yend,N);								// Reference (y)
    for(int i = 0; i!=N; i++){													// Reference (v)
        ref.v.push_back(state[4]);
    }
    ref.dir = 1;																// fwd driving only
    // Initialize tree
    vector<state_type> T; T.push_back(state);
    Node initialNode(state,-1,ref,T,0,0,0);
	tree.push_back(initialNode);
}

double initializeTree(MyRRT& RRT, const Vehicle& veh, vector<Path>& path, vector<double> carState){
	carState.push_back(0); carState.push_back(0); carState.push_back(0); carState.push_back(0); 
	// assert(carState.size()==8);	// Make sure the state is correct size (add time & IDwp)
	double Tp = 0;
	// If committed path is empty, initialize tree with first node at lookahead point
	if (path.size()==0){
		RRT.addInitialNode(carState);
		ROS_INFO_STREAM("Initialized empty tree.");
		return Tp;
	}

	// See which parts of committed path have been passed and erase these from the pathlist
	// 1. Loop through the segments
	// 2. erase everything behind the car
	geometry_msgs::Point Ppreview; Ppreview.x = ctrl_dla; Ppreview.y = 0; Ppreview.z=0;
	for(auto it = path.begin(); it!=path.end(); ++it){
		if ((it->tra.back()[0])<0){
			path.erase(it--);
			cout<<"Removed part of plan."<<endl;
		}else{
			for(auto it2 = it->tra.begin(); it2!=it->tra.end(); ++it2){
				if (((*it2)[0])<0){
					it->tra.erase(it2--);
				}
			}
		}
	}
	MyReference refMerged;
	for(auto it = path.begin(); it!=path.end(); ++it){
		refMerged.dir = 1;
		for(int i = 0; i != it->tra.size(); i++){
			refMerged.x.push_back(it->ref.x[i]);
			refMerged.y.push_back(it->ref.y[i]);
			refMerged.v.push_back(it->ref.v[i]);
		}
	}
	Simulation sim(RRT,carState,refMerged,veh,false,false);
	Tp = sim_dt*(sim.stateArray.size()-1);

	// If this assertion fails, the path commitment is not configured properly
	if (Tp<0.1){
		ROS_ERROR_STREAM("Committed time is less than it takes to plan a path, reconfigure this!");
		assert(Tp>=0.1);
	}

	// Initialize tree
	Node node(sim.stateArray.back(), -1, refMerged, sim.stateArray,0,0,0);
	node.state[6] = Tp;
	RRT.tree.push_back(node);
	cout<<"Initialized tree with last committed reference."<<endl;

	// // Calculate total committed time
	// for(auto it = path.begin(); it!=path.end(); ++it){
	// 	for(int j = 1; j<it->tra.size(); j++){
	// 		Tc += sim_dt;
	// 	}
	// }
	return Tp;
}


// Perform a tree expansion
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub, const vector<car_msgs::Obstacle2D>& det, const vector<double>& Cxy){;
	// #### RANDOM SAMPLING: ####
	geometry_msgs::Point sample;
	if (RRT.bend){ 	// Sample on lane
		double Lmax = RRT.goalPose[0];
		sample = sampleOnLane(Cxy,RRT.laneShifts, Lmax);
	}else{ 			// Sample around vehicle
		vector<double> bounds = {0,RRT.goalPose[0]+5,RRT.goalPose[1]-3, RRT.goalPose[1]+3};
		sample = sampleAroundVehicle(bounds);
	}
	signed int dir = 1; // Driving direction variable
	// #### SORTING THE NODES ####
	// P1: sort nodes according to increasing Dubins distance to sample
	// P2: sort nodes according to total cost (time) to reach the sample 
	vector<int> sortedNodes; bool node_added = false;
	double r = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(1))); 	// Generate random value [0-1]
	// if(r<=((RRT.goalReached*0.3)+(!RRT.goalReached*0.7))){								// Select a heuristic (shifts after goal is reached)
	// 	sortedNodes = sortNodesExplore(RRT,sample); 		// Sort nodes in increasing Dubins distance to sample
	// }else{														
		sortedNodes = sortNodesOptimize(RRT,sample); 		// Sort nodes on total cost (time) to reach sample
	// }
	// #### NODE EXPANSION ####
	// Loop through the sorted nodes untill expansion succeeds
	for(vector<int>::iterator it = sortedNodes.begin(); it != sortedNodes.end(); ++it){		
		MyReference ref = getReference(sample, RRT.tree[*it], dir);	// Generate a reference path
		Simulation sim(RRT,RRT.tree[*it].state,ref,veh,false,true);					// Do closed-loop prediction
		// If trajectory is admissible and collisionfree, add it to the tree
		if(sim.endReached||sim.goalReached){
			if (!checkCollision(ptrPub,sim.stateArray,det)){
				Node node(sim.stateArray.back(), *it, ref,sim.stateArray, sim.costE + RRT.tree[*it].costE, sim.costS + RRT.tree[*it].costS, sim.goalReached);
				RRT.addNode(node); 	node_added = true;
				break;
			}else{
				fail_collision++;
			}
		}
	}; 
	// #### GOAL BIASED EXPANSION ####
	// Loop through the added nodes and try a goal expansion
	if ( node_added && feasibleGoalBias(RRT) ) { 
		if(debug_mode){cout<<"Doing goal expansion..."<<endl;}
		MyReference ref_goal = getGoalReference(veh, RRT.tree.back(), RRT.goalPose);
		Simulation sim_goal(RRT,RRT.tree.back().state, ref_goal,veh,true,true);				
		
		// If trajectory is admissible and collision free, add it to the tree
		if(sim_goal.endReached||sim_goal.goalReached){
			if(!checkCollision(ptrPub,sim_goal.stateArray,det)){
				Node node_goal(sim_goal.stateArray.back(), RRT.tree.size()-1, ref_goal, sim_goal.stateArray,sim_goal.costE + RRT.tree.back().costE, sim_goal.costS + RRT.tree.back().costS, sim_goal.goalReached);
				RRT.addNode(node_goal);
			}else{
				fail_collision++;
			}
		}
	}
};

// Uniform sampling around the vehicle
geometry_msgs::Point sampleAroundVehicle(vector<double> sampleBounds){
	geometry_msgs::Point sample;	
	sample.x = sampleBounds[0] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(sampleBounds[1]-sampleBounds[0])));
	sample.y = sampleBounds[2] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(sampleBounds[3]-sampleBounds[2])));
	if(debug_mode){cout<<"Generated sample: x="<<sample.x<<" y="<<sample.y<<endl;}
	return sample;
}

// Sample on the given lane center lines
geometry_msgs::Point sampleOnLane(const vector<double>& Cxy, vector<double> laneShifts, double Lmax){
	// sample w.r.t. the straightened road y(x) = c1*x + c0;
	// 1. Sample length coordinate (S) on reference road centerline
	double S = ctrl_dla + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(Lmax-ctrl_dla)));
	// 2. Sample coordinate (rho) from lane shifts
	// Select a random lane
	double r = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(((laneShifts.size()-1)))));
	int laneIndex = (int) floor(r+0.5);	
	double rho = laneShifts[laneIndex];
	assert(0<=laneIndex<=(laneShifts.size()-1));
    // Rotate (S,rho) with slope, translate with C0
	double theta = atan2(Cxy[1], 1);
	double Xstraight = cos(theta)*S - sin(theta)*rho;
	double Ystraight = sin(theta)*S + cos(theta)*rho + Cxy[2];
	// Prepare sample
	geometry_msgs::Point sample;
	sample.x = Xstraight;
	sample.y = Ystraight;
	if(debug_mode){cout<<"Generated sample: x="<<sample.x<<" y="<<sample.y<<endl;}
	return sample;
}

// Sort nodes according to the exploration heuristic (Dubins distance)
vector<int> sortNodesExplore(const MyRRT& rrt, const geometry_msgs::Point& sample){
	vector<pair<int,float>> dVector;
	// Generate data pair of ID + Dubins distance
	for(int nodeid = 0; nodeid != rrt.tree.size(); nodeid++){
		dVector.push_back(make_pair(nodeid,dubinsDistance(sample, rrt.tree[nodeid], rrt.direction)));
	};
	// Sort the pairs from shortest to longest distance
	sort(dVector.begin(),dVector.end(),[](const pair<int,float>& a, const pair<int,float>& b){return a.second< b.second;});
	// Extract feasible connections until maximum size is reached
	vector<int> sortedList;
	for(vector<pair<int,float>>::iterator it = dVector.begin(); it != dVector.end(); ++it){
		if(feasibleNode(rrt,rrt.tree[it->first],sample))
		{
			sortedList.push_back(it->first);
		}
		if (sortedList.size()==rrt.sortLimit){break;}
	}

	if(debug_mode){cout<<"sorted nodes to exploring heuristic."<<endl;}
	return sortedList;
}

// Sort nodes according to the optimization heuristic (Travel time)
vector<int> sortNodesOptimize(const MyRRT& rrt, const geometry_msgs::Point& sample){
	vector<pair<int,float>> dVector;
	for(int index = 0; index != rrt.tree.size(); index++){
		dVector.push_back(make_pair(index,rrt.tree[index].costE + dubinsDistance(sample, rrt.tree[index], rrt.direction)));
	};
	sort(dVector.begin(),dVector.end(),[](const pair<int,float>& a, const pair<int,float>& b){return a.second< b.second;});
	vector<int> sortedList;
	for(vector<pair<int,float>>::iterator it = dVector.begin(); it != dVector.end(); ++it){
		if(feasibleNode(rrt,rrt.tree[it->first],sample))
		{
			sortedList.push_back(it->first);
		}
		if (sortedList.size()==rrt.sortLimit){break;}
	}
	
	if(debug_mode){cout<<"Sorted nodes with optimization heuristic."<<endl;}
	return sortedList;
}

// Check if node connection is feasible
bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample){
	// Calculate reference heading
	double angPar = atan2(node.ref.y.back()-node.ref.y.front(),node.ref.x.back()-node.ref.x.front());
	double angNew = atan2(sample.y-node.ref.y.back(),sample.x-node.ref.x.back());
	// angNew = angNew + (node.ref.dir*rrt.direction<0)*pi; // Required when reverse driving is implemented
	// Calculate length of new reference
	double Lref = sqrt( pow(node.ref.x.back()-sample.x,2) + pow(node.ref.y.back()-sample.y,2));
	// Reject when heading difference exceeds limit
	if (abs(angleDiff(angNew,angPar))>(pi/4)){
        return false;
    }
	// Reject when new reference would be too short (at least 3 data points)
    else if(Lref<(2.1*ref_res)){
		return false;
    }
	else{
		return true;
	}
}

// Check if a goal biased expansion is feasible
bool feasibleGoalBias(const MyRRT& rrt){
	// Define circles of minimum turning radius left and right of the vehicle
	double R1{4.77}; double R2{R1-0.3};
	geometry_msgs::Point center_l, center_r;
	center_l.x = rrt.goalPose[0]+R1*cos(rrt.goalPose[2]-M_PI_2);
	center_l.y = rrt.goalPose[1]+R1*cos(rrt.goalPose[2]-M_PI_2);
	center_r.x = rrt.goalPose[0]+R1*cos(rrt.goalPose[2]+M_PI_2);
	center_r.y = rrt.goalPose[1]+R1*cos(rrt.goalPose[2]+M_PI_2);
	// If the node state lies within either one of these circles, the goal bias is not feasible due to the vehicle' minimum turning radius
	Node node = rrt.tree.back();
	bool outside_left_circle = sqrt( pow(node.state[0]-center_l.x,2) + pow(node.state[1]-center_l.y,2) ) > R2;
	bool outside_right_circle = sqrt( pow(node.state[0]-center_r.x,2) + pow(node.state[1]-center_r.y,2) ) > R2;
	// Determine the angle of the reference to the goal heading
	double angleRef = atan2( rrt.goalPose[1]-node.ref.y.back(), rrt.goalPose[0]-node.ref.x.back());
	double dHead1 = abs(wrapToPi(rrt.goalPose[2]-angleRef)); 
	double dHead2 = abs(wrapToPi(rrt.goalPose[2]+pi-angleRef));
	double minAngleDiff = min(dHead1,dHead2);
	double sgn = sign(cos(rrt.goalPose[2]+M_PI_2-angleRef));
	double angle = sgn*minAngleDiff;
	// Check if angle lies is within limits
	bool angle_within_limits = abs(angle)<(M_PI_4/2);

	return outside_left_circle*outside_right_circle*angle_within_limits;
}

// Extract the best path from the tree with backtracking
vector<Node> extractBestPath(vector<Node> tree, ros::Publisher* ptrPub){  
	vector<Node> bestPath; 					// Initialize returned vector
	vector<pair<int,double>> pair_vector;	// Initialize pair. 1: NodeID, 2: Cost
	// Loop through the tree. When node reached goal, add it to the pair vector
	visualization_msgs::MarkerArray msgArray;
	visualization_msgs::Marker msgClear = createEmptyMsg();
	msgArray.markers.push_back(msgClear);
	for(int nodeid = 0; nodeid !=tree.size(); nodeid++){
		if(tree[nodeid].goalReached){
				pair_vector.push_back(make_pair(nodeid,tree[nodeid].costS));
				if(draw_tree){
					visualization_msgs::Marker msg = createStateMsg(nodeid,tree[nodeid].tra,1);
					msgArray.markers.push_back(msg);
				}				
		}else{
			if (draw_tree){
				visualization_msgs::Marker msg = createStateMsg(nodeid,tree[nodeid].tra,0);
				msgArray.markers.push_back(msg);
			}
		}
	}
	if(draw_tree){
		ptrPub->publish(msgArray);
	}
	if(pair_vector.size()==0){
		ROS_WARN("No solution was found!");
	}else{
		cout<<pair_vector.size()<<" paths to the goal found!"<<endl;
		// Sort the pair vector in ascending cost
		sort(pair_vector.begin(),pair_vector.end(),[](const pair<int,double>& a, const pair<int,double>& b){return a.second< b.second;});
		// Add lowest cost solution to best path vector
		bestPath.push_back(tree[pair_vector.front().first]);
		// Backtracking
		int parent = bestPath.front().parentID;
		while (parent!=0){
			bestPath.insert(bestPath.begin(), tree[parent]);
			parent = bestPath.front().parentID;
		}
	}
	return bestPath;
}

// Calculate Dubins distance between a pose (R2S) and point (R2)
float dubinsDistance(geometry_msgs::Point S, Node N, int dir){
    // Distance measurement with the Dubins metric
    float rho = 4.77;
    // 1. Subtract node location
    float qw_x = S.x - N.state[0];
    float qw_y = S.y - N.state[1];
    // 2. Rotate to 0 rotation
    float ang = -N.state[2]-M_PI*(dir!=1);
    float tmp = cos(ang)*qw_x - sin(ang)*qw_y;
    qw_y = abs(sin(ang)*qw_x + cos(ang)*qw_y);
    qw_x = tmp;

    // Parts of the solution
    float dc = sqrt( qw_x*qw_x + (qw_y-rho)*(qw_y-rho) );
    float thetac = atan2(qw_x,rho-qw_y);
    while(thetac<0){
        thetac = thetac + 2*M_PI;
    }

    float df = sqrt( qw_x*qw_x + (qw_y+rho)*(qw_y+rho) );
    float alpha =  2*M_PI - acos( (5*rho*rho - df*df)/(4*rho*rho));

    // Check if qw lies within circles
    bool q_in_Dp = 0;
    if ((qw_x*qw_x+(qw_y+rho)*(qw_y+rho)<=rho*rho)|(qw_x*qw_x+(qw_y-rho)*(qw_y-rho)<=rho*rho)){
        q_in_Dp = 1;
    }

    // Choose solution
    if(!q_in_Dp){
        return sqrt(dc*dc - rho*rho) + rho*(thetac - acos(rho/dc));
    }
    else{
        return rho*(alpha + asin(qw_x/df) - asin(rho*sin(alpha)/df));
    }
}

/******************************************
 **** RVIZ PUBLISHING *********************
 *****************************************/

// Create a message for publishing a trajectory
visualization_msgs::Marker createStateMsg(int ID, const vector<vector<double>> T, bool goalReached){
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "trajectory";
	msg.id = ID;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.05;	// msg/LINE_LIST markers use only the x component of scale, for the line width
	if (goalReached){
		msg.color.g = 1;
		msg.color.r = 1;
	}else{
		msg.color.r = 1.0;
	}
	msg.color.a = 1.0;
	msg.lifetime = ros::Duration();
    
    geometry_msgs::Point p;
    for(int i = 0; i<T.size(); i++){
        p.x = T[i][0];
        p.y = T[i][1];
        p.z = 0;
        msg.points.push_back(p);
    }
    return msg;    
}

// Create message for deleting all Rviz markers
visualization_msgs::Marker createEmptyMsg(){
    // Initialize marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "center_laser_link";
    msg.header.stamp = ros::Time::now();
    msg.ns = "trajectory";
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    return msg;    
}
