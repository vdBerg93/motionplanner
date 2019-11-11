#include <cmath>

#include "rrt/controller.h"
#include "rrt/simulation.h"
#include "rrt/controller.h"

void enforceConstraints(const double& min, const double& max, double& val){
    val = std::max(std::min(val,max),min);
}

state_type VehicleODE(ControlCommand& ctrl, state_type& x, const Vehicle& veh){
    state_type dx(7);
	double Gss = 1/( 1 + pow((x[4]/20),2)); 	// Sideslip transfer function
	dx[0] = x[4]*cos(x[2]);            			// xdot
    dx[1] = x[4]*sin(x[2]);            			// ydot
    dx[2] = (x[4]/2.885)*tan(x[3])*Gss;			// thetadot
    dx[3] = (1/0.3)*(ctrl.dc-x[3]);  			// deltadot
    dx[4] = x[5];                      			// vdot
    dx[5] = (1/0.3)*(ctrl.ac-x[5]);  			// adot
    dx[6] = 1;                          		// dt
	// Constraints
	enforceConstraints(veh.amin, veh.amax, dx[4]);
	enforceConstraints(-veh.ddmax, veh.ddmax, dx[3]);
	return dx;
};

void IntegrateEuler(ControlCommand& ctrl, state_type& x, state_type& dx, double& dt, const Vehicle& veh){
	for(int i = 0; i<= x.size(); i++){
		x[i] = x[i] + dx[i]*dt;
	};
	// Constraints
	enforceConstraints(-veh.dmax,veh.dmax,x[3]);
	enforceConstraints(veh.amin, veh.amax, dx[5]);
};

Simulation::Simulation(const MyRRT& RRT, const vector<double>& state, MyReference& ref, const Vehicle& veh, bool GoalBiased): 
	costE(0), costS(0), goalReached(false), endReached(false){
	stateArray.push_back(state); 				// Push initial state into statearray
	Controller control(ref,state);				// Initialize controller
	stateArray.back()[7] = control.IDwp;		// Add waypoint ID in stateArray
	generateVelocityProfile(ref,control.IDwp,state[4],vmax,RRT.goalPose,GoalBiased);
	propagate(RRT, control,ref,veh);			// Predict vehicle trajectory
};

double getDistToLane(const double& x, const double& y, double S, const vector<double>& Cxy){
	double Lx = (x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2])/(pow(Cxy[1],2) + 1);
	double Ly = S + Cxy[2] + (Cxy[1]*(x - S*Cxy[1] + y*Cxy[1] - Cxy[1]*Cxy[2]))/(pow(Cxy[1],2) + 1);
	return sqrt( pow(Lx-x,2) + pow(Ly-y,2) );
}

void Simulation::propagate(const MyRRT& RRT, Controller control, const MyReference& ref, const Vehicle& veh){
	// int w1 = 160; int w2 = 80; int wc = 2;
	int Wlane = 1;
	int Wcurv = 4000;
	if(draw_states){
		cout<<"-------------------------------------------------------------"<<endl;
		cout<<"reference size= "<<ref.x.size()<<endl;
		cout<<"[ "<<ref.x.front()<<" -> "<<ref.x.back()<<" ]"<<endl;
		cout<<"[ "<<ref.y.front()<<" -> "<<ref.y.back()<<" ]"<<endl;
		cout<<"-------------------------------------------------------------"<<endl;
	}
	for(int i = 0; i<(20/sim_dt); i++){
		state_type x = stateArray[i];								// Set x as last vehicle state
		ControlCommand ctrlCmd = control.getControls(ref,veh,x);	// Get controls for state
		state_type dx = VehicleODE(ctrlCmd, x, veh);				// Get vehicle state transition
		IntegrateEuler(ctrlCmd, x, dx, sim_dt, veh);				// Get new state
		x[7] = control.IDwp;									// Add waypoint ID to vehicle state
		stateArray.push_back(x);									// Add state to statearray
		// costE = costE + x[4]*sim_dt; 								// Update cost estimate for exploration
		costE += sim_dt;
		double kappa = tan(x[3])/veh.L;								// Calculate vehicle path curvature
		if (RRT.bend){
			ROS_WARN_STREAM_ONCE("Goal lane shift= "<<RRT.laneShifts[0]);
			double Dgoallane = getDistToLane(x[0],x[1],RRT.laneShifts[0],RRT.Cxy);
			double Dotherlane = getDistToLane(x[0],x[1],RRT.laneShifts[1],RRT.Cxy);
			// costS += (Dotherlane<Dgoallane)*w1*abs(kappa) +		// Less cost on curvature in first lane
			// 		(Dgoallane<Dotherlane)*w2*abs(kappa) + 		// More cost on curvature in next lane
			// 		wc*Dgoallane;
			costS += Dgoallane;
		}else{
			// costS += sim_dt + 10*kappa;
			// costS += kappa;
			// costS += x[4]*sim_dt;
			costS += sim_dt + 0.2*kappa;
		}

		// Check acceleration limits
		double ay = abs(x[4]*dx[2]);
		ROS_WARN_STREAM_THROTTLE(2,"Max road lateral acceleration: "<<ay_road_max);
		if ( ay + ay_road_max> 3){
			if(debug_sim){	ROS_WARN_STREAM("Acceleration exceeded! "<<ay<<" m/s2 , delta="<<x[3]);}
			endReached = false; return;
		}
		if (draw_states){
			// Print the states
			cout<<"x="<<stateArray.back()[0]<<",\ty="<<stateArray.back()[1]<<",\thead="<<stateArray.back()[2]<<",\td="<<stateArray.back()[3]<<",\tv="<<stateArray.back()[4]<<",\ta="<<stateArray.back()[5]
			// Print the control variables
			<<",\tt="<<stateArray.back()[6]<<",\tIDwp="<<stateArray.back()[7]<<", \t\t\tWpx="<<control.Ppreview.x<<", \t WPy="<<control.Ppreview.y<<",\tvcmd="<<ref.v[control.IDwp]<<",\tym="<<control.ym<<endl;
		}

		// Stop simulation when end of reference is reached and velocity < terminate velocity
		double Verror = (x[4]-ref.v.back());
		if (control.endreached&&(Verror<0.1)){
			if(debug_sim){	ROS_INFO_STREAM("end reached");}
			endReached = true; return;
		}
		// Stop simulation if goal is reached
		double dist_to_goal = sqrt( pow(x[0]-RRT.goalPose[0],2) + pow(x[1]-RRT.goalPose[1],2));
		double goal_heading_error = abs(angleDiff(x[2],RRT.goalPose[2]));
		
		if ((dist_to_goal<=1)&&(goal_heading_error<0.1)){
			double Verror = (x[4]-RRT.goalPose[3]);
			if(debug_mode){ROS_WARN_STREAM("Near goal! Vel error= "<<Verror);}
			if (Verror<0.05){
				if(debug_sim){	ROS_INFO_STREAM("goal reached");}
				goalReached = true; return;
			}
		}
	}	
};
