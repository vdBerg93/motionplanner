#include <limits>
#include "rrt/collision.h"
#include "rrt/rrtplanner.h"
#include "rrt/simulation.h"

using namespace std;

void publishVisualization(ros::Publisher* ptrPub, int ID, MyReference& ref, Simulation sim);

MyRRT::MyRRT(const vector<double>& _goalPose, const vector<double>& _laneShifts, const vector<double>& _Cxy){
	goalReached = 0;
	sortLimit = 10;
	direction = 1;
    goalPose = _goalPose;
	laneShifts = _laneShifts;
	Cxy = _Cxy;
}

void MyRRT::addInitialNode(const vector<double>& state){
	// Generate initial reference
	MyReference ref; 
    double xend = state[0]+cos(state[2])*ctrl_dla;
    double yend = state[1]+sin(state[2])*ctrl_dla;
    int N = floor( sqrt( pow(state[0]-xend,2) + pow(state[1]-yend,2) )/ref_res);
    ref.x = LinearSpacedVector(state[0],xend,N);
    ref.y = LinearSpacedVector(state[0],yend,N);
	// ref.x.push_back(xend);
	// ref.y.push_back(yend);
    for(int i = 0; i!=N; i++){
        ref.v.push_back(state[4]);
    }
    ref.dir = 1;
    // Initialize tree
    vector<state_type> T; T.push_back(state);
    Node initialNode(state,-1,ref,T,0,0,0);
	tree.push_back(initialNode);
}

double initializeTree(MyRRT& RRT, const Vehicle& veh, vector<MyReference>& path, const vector<double>& carState){
	double Tc {0};
	// If committed path is empty, initialize tree with reference at (Dla,0)
	if (path.size()==0){
		RRT.addInitialNode(carState);
		ROS_INFO_STREAM("Initialized empty tree.");
		return Tc;
	}

	// See which parts of path have been passed and erase these from the pathlist
	geometry_msgs::Point Ppreview; Ppreview.x = ctrl_dla; Ppreview.y = 0; Ppreview.z=0;
	for(auto it = path.begin(); it!=path.end(); ++it){
		assert(it->x.size()>=3);
		int ID = findClosestPoint(*it,Ppreview,1);
		if ((path.size()>1)&&(ID >= (it->x.size()-3))){
			path.erase(it--);
		}
	}



	// For the rest: propagate states to obtain nodes
	vector<Node> nodeList;
	for(auto it = path.begin(); it!=path.end(); ++it){
			if(nodeList.size()==0){
				Simulation sim(RRT,carState,*it,veh);
				Node node(sim.stateArray.back(), -1, *it,sim.stateArray, sim.costE, sim.costS, sim.goalReached);
				nodeList.push_back(node);
				Tc += sim.stateArray.size()*sim_dt;
			}else{
				Simulation sim(RRT,RRT.tree.back().state,*it,veh);		
				Node node(sim.stateArray.back(), -1, *it,sim.stateArray, sim.costE + RRT.tree.back().costE, sim.costS + RRT.tree.back().costS, sim.goalReached);
				nodeList.push_back(node);
				Tc += sim.stateArray.size()*sim_dt;
			}
	}
	// Add last node as first node to tree
	RRT.tree.push_back(nodeList.back());
	ROS_INFO_STREAM("Initialized tree with last committed reference.");
	return Tc;
}


// Perform a tree expansion
void expandTree(Vehicle& veh, MyRRT& RRT, ros::Publisher* ptrPub, const vision_msgs::Detection2DArray& det, const vector<double>& Cxy){;
	double Lmax = RRT.goalPose[0];// RRT.goalPose[0]
	geometry_msgs::Point sample = sampleOnLane(Cxy,RRT.laneShifts, Lmax);
	// ROS_WARN_STREAM_ONCE("Sampledomain:"<<double(0)<<", "<<Lmax<<"-"<<RRT.laneShifts[0]<<", "<<RRT.laneShifts[1]);
	// cout<<"Sx= "<<sample.x<<" s= "<<sample.y<<endl;
	// vector<double> bounds = {0,150,-50,50};
	// geometry_msgs::Point sample = sampleAroundVehicle(bounds);
	signed int dir = 1; 		// Driving direction variable
	
	// Sort existing nodes using randomly selected Heuristics
	vector<int> sortedNodes; bool node_added = false;
	double r = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(1))); // Generate random value [0-1]
	if(r<=((RRT.goalReached*0.3)+(!RRT.goalReached*0.7))){	// Probability shifts after goal is reached	
		sortedNodes = sortNodesExplore(RRT,sample); 		// Sort nodes in increasing Dubins distance to sample
	}else{														
		sortedNodes = sortNodesOptimize(RRT,sample); 		// Sort nodes on total cost to reach sample
	}

	// Loop through the sorted nodes untill expansion succeeds
	for(vector<int>::iterator it = sortedNodes.begin(); it != sortedNodes.end(); ++it){
		if(debug_mode){cout<<endl<<"Trying to expand node "<<*it<<endl;}
		
		MyReference ref = getReference(sample, RRT.tree[*it], dir);	// Generate a reference path
		Simulation sim(RRT,RRT.tree[*it].state,ref,veh);					// Do closed-loop prediction

		if(debug_mode){cout<<"completed simulation."<<endl;};

		// If trajectory is admissible and collisionfree, add it to the tree
		if(sim.endReached||sim.goalReached){
			if (!checkCollision(ptrPub,sim.stateArray,det)){
				Node node(sim.stateArray.back(), *it, ref,sim.stateArray, sim.costE + RRT.tree[*it].costE, sim.costS + RRT.tree[*it].costS, sim.goalReached);
				RRT.addNode(node); 	node_added = true;
				if(draw_tree){publishVisualization(ptrPub, RRT.tree.size(), ref, sim);}
				break;
			}
		}
	}; 

	// Loop through the added nodes and try a goal expansion
	if ( node_added && feasibleGoalBias(RRT) ) { 
		if(debug_mode){cout<<"Doing goal expansion..."<<endl;}
		MyReference ref_goal = getGoalReference(veh, RRT.tree.back(), RRT.goalPose);
		Simulation sim_goal(RRT,RRT.tree.back().state, ref_goal,veh);				
		
		// If trajectory is admissible and collision free, add it to the tree
		if(sim_goal.endReached||sim_goal.goalReached){
			if(!checkCollision(ptrPub,sim_goal.stateArray,det)){
				Node node_goal(sim_goal.stateArray.back(), RRT.tree.size()-1, ref_goal, sim_goal.stateArray,sim_goal.costE + RRT.tree.back().costE, sim_goal.costS + RRT.tree.back().costS, sim_goal.goalReached);
				RRT.addNode(node_goal);
				if(draw_tree){publishVisualization(ptrPub, RRT.tree.size(), ref_goal, sim_goal);}
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

geometry_msgs::Point sampleOnLane(const vector<double>& Cxy, vector<double> laneShifts, double Lmax){
	// If no lane information is available, sample around the vehicle
	vector<double> sampleBounds {0,150,-10,10};
	if(Cxy.size()==0){
		return sampleAroundVehicle(sampleBounds);
	}
	// Else, sample w.r.t. the straightened road y(x) = c1*x + c0;
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

bool feasibleNode(const MyRRT& rrt, const Node& node, const geometry_msgs::Point& sample){
	// Calculate reference heading
	double angPar = atan2(node.ref.y.back()-node.ref.y.front(),node.ref.x.back()-node.ref.x.front());
	double angNew = atan2(sample.y-node.ref.y.back(),sample.x-node.ref.x.back());
	// angNew = angNew + (node.ref.dir*rrt.direction<0)*pi;
	// Calculate length of new reference
	double Lref = sqrt( pow(node.ref.x.back()-sample.x,2) + pow(node.ref.y.back()-sample.y,2));
	// Reject when heading difference exceeds limit
	if (abs(angleDiff(angNew,angPar))>(pi/8)){
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

vector<Node> extractBestPath(vector<Node> tree, bool structured){  
	vector<Node> bestPath; 		// Initialize returned vector
	vector<pair<int,double>> pair_vector;	// Initialize pair. 1: NodeID, 2: Cost
	// Loop through the tree. When node reached goal, add it to the pair vector
	for(int nodeid = 0; nodeid !=tree.size(); nodeid++){
		if(tree[nodeid].goalReached){
			if(structured){
				pair_vector.push_back(make_pair(nodeid,tree[nodeid].costS));
			}
		}
	}
	if(pair_vector.size()==0){
		ROS_WARN("No solution was found!");
	}else{
		// Sort the pair vector in ascending cost
		sort(pair_vector.begin(),pair_vector.end(),[](const pair<int,double>& a, const pair<int,double>& b){return a.second< b.second;});
		// Add lowest cost solution to best path vector
		bestPath.push_back(tree[pair_vector.front().first]);
		// Backtracking
		int parent = bestPath.front().parentID;
		for(int i = 0; i<tree.size(); i++){
			bestPath.insert(bestPath.begin(), tree[parent]);
			parent = bestPath.front().parentID;
			// Stop when root node is reached
			if(parent==(0)){
				return bestPath;
			}
		}
	}
	return bestPath;
}

void publishVisualization(ros::Publisher* ptrPub, int ID, MyReference& ref, Simulation sim){
	visualization_msgs::Marker msg = createReferenceMsg(ID,ref);
	ptrPub->publish(msg);	// Publish reference path to Rviz (Yellow)
	msg = createStateMsg(ID,sim.stateArray);
	ptrPub->publish(msg);	// Publish trajectory to Rviz (Green)
}

visualization_msgs::Marker createReferenceMsg(int iD, const MyReference& ref){
    // Initialize marker message
    static visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "reference";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;

    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.scale.x = 0.05;	// msg/LINE_LIST markers use only the x component of scale, for the line width

    // Line strip is blue
    msg.color.r = 1.0;
    msg.color.b = 0.0;
    msg.color.g = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration();
    
    geometry_msgs::Point p;
    for(int i = 0; i<ref.x.size(); i++){
        p.x = ref.x[i];
        p.y = ref.y[i];
        p.z = 0;
        msg.points.push_back(p);
    }
    
    return msg;    
}

visualization_msgs::Marker createStateMsg(int ID, const vector<vector<double>> T){
    // Initialize marker message
    static visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "trajectory";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.scale.x = 0.2;	// msg/LINE_LIST markers use only the x component of scale, for the line width

    // Line strip is green
    msg.color.g = 1.0;
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