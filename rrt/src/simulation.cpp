#include <cmath>

#include "rrt/controller.h"
#include "rrt/simulation.h"
#include "rrt/controller.h"

void VehicleODE(ControlCommand& ctrl, state_type& x, state_type& dx){
    double Gss = 1/( 1 + pow((x[4]/20),2)); 	// Sideslip transfer function
	dx[0] = x[4]*cos(x[2]);            			// xdot
    dx[1] = x[4]*sin(x[2]);            			// ydot
    dx[2] = (x[4]/2.885)*tan(x[3])*Gss;			// thetadot
    dx[3] = (1/0.3)*(ctrl.dc-x[3]);  			// deltadot
    dx[4] = x[5];                      			// vdot
    dx[5] = (1/0.3)*(ctrl.ac-x[5]);  			// adot
    dx[6] = 1;                          		// dt
};

void IntegrateEuler(ControlCommand& ctrl, state_type& x, double& dt){
	state_type dx(7);
	VehicleODE(ctrl, x, dx);
	for(int i = 0; i<= x.size(); i++){
		x[i] = x[i] + dx[i]*dt;
	};
};

Simulation::Simulation(const MyRRT& RRT, Node& node, MyReference& ref, Vehicle& veh){
	costE = costS = goalReached = endReached = 0;
	stateArray.push_back(node.state);
	Controller control(ref,node.state);
	generateVelocityProfile(ref,node,control.IDwp,vmax,vgoal);
	propagate(RRT, control,ref,veh);
};

void Simulation::propagate(const MyRRT& RRT, Controller control, const MyReference& ref, const Vehicle& veh){
	double kappa;
	// int w1 = 160; int w2 = 80; int wc = 2;
	int w1 = 160; int w2 = 80; int wc = 5;
	state_type x(7), dx(7);

	for(int i = 0; i<(20/sim_dt); i++){
		x = stateArray[i];
		ControlCommand ctrlCmd = control.getControls(ref,veh,x);
		IntegrateEuler(ctrlCmd, x, sim_dt);
		stateArray.push_back(x);
		ROS_WARN_ONCE("TODO: In simulation: make lanewidth variable.");
		double LaneWidth {3.5}; 
		// Update cost estimate for exploration
		costE = costE + x[4]*sim_dt;
		// Update cost estimate for path selection
		kappa = tan(x[3])/veh.L;
		costS += (x[1]<0.5*LaneWidth)*w1*abs(kappa) +		// Less cost on curvature in first lane
				(x[1]>=0.5*LaneWidth)*w2*abs(kappa) + 		// More cost on curvature in next lane
				wc*abs(min(x[1],abs(x[1]-LaneWidth)));		// Cost on deviation from closest centerline
		// Check acceleration limits
		if (x[2]*abs(x[4])>2){
			endReached = false; return;
		}
		if (draw_states){
			// Print the states
			cout<<"x="<<stateArray.back()[0]<<",\ty="<<stateArray.back()[1]<<",\thead="<<stateArray.back()[2]<<",\td="<<stateArray.back()[3]<<",\tv="<<stateArray.back()[4]<<",\ta="<<stateArray.back()[5]
			// Print the control variables
			<<",\tt="<<stateArray.back()[6]<<",\t\t\tIDwp="<<control.IDwp<<", \tWpx="<<control.Ppreview.x<<", \t WPy="<<control.Ppreview.y<<",\tvcmd="<<ref.v[control.IDwp]<<",\tym="<<control.ym<<endl;
		}

		// Stop simulation when end of reference is reached and velocity < terminate velocity
		double term_velocity = 0.1;
		if((control.endreached)&&(abs(x[4]-ref.v.back())<term_velocity)){
			endReached = true; return;
		}
		// Stop simulation if goal is reached
		double dist_to_goal = sqrt( pow(x[0]-RRT.goalPose[0],2) + pow(x[1]-RRT.goalPose[1],2));
		// double goal_heading_error = angleDiff(x[2],RRT.goalPose[2]);
		double goal_heading_error = abs(x[2]-RRT.goalPose[2]);
		if ((dist_to_goal<=0.3)&&(goal_heading_error<0.05)){
			goalReached = true; return;
		}
	}

};