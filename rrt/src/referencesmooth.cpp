
/* --------------------------------------
	REFERENCE GENERATION
---------------------------------------*/
#include "rrt/rrtplanner.h"
#include "ros/ros.h"

// Generate a linear reference paths
MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir){
	MyReference ref;
	double L = sqrt( pow(sample.x-node.ref.x.back(),2) + pow(sample.y-node.ref.y.back(),2) );
	int N = round(L/ref_res)+1;
	ref.x = LinearSpacedVector(node.ref.x.back(),sample.x,N);
	ref.y = LinearSpacedVector(node.ref.y.back(),sample.y,N);
	ref.dir = dir;
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated reference."<<endl;}
	return ref;
};

// Generate a goal biased reference
MyReference getGoalReference(const Vehicle& veh, Node node, vector<double> goalPose){;
	double Dextend = ctrl_dla;//+1.2;
	double Dalign = 1;
	
	MyReference ref;

	// Alignment
	geometry_msgs::Point P1, P2, Pclose, Pfar;
	P1.x = goalPose[0]+Dalign*cos(goalPose[2]); P1.y = goalPose[1]+Dalign*sin(goalPose[2]);
	P2.x = goalPose[0]-Dalign*cos(goalPose[2]); P2.y = goalPose[1]-Dalign*sin(goalPose[2]);
	double H = atan2(P2.y-P1.y,P2.x-P1.x);

	// Select closest point
	if( sqrt( pow(P1.x-node.ref.x.back(),2) + pow(P1.y-node.ref.y.back(),2)) < sqrt( pow(P2.x-node.ref.x.back(),2) + pow(P2.y-node.ref.y.back(),2))){
		Pclose = P1; Pfar = P2; 
	}else{
		Pclose = P2; Pfar = P1;
	}
	// Extend to account for lookahead distance
	Pfar.x += Dextend*cos(goalPose[2]);
	Pfar.y += Dextend*sin(goalPose[2]);
	
	// Segment lengths
	double N1 = round(sqrt( pow(Pclose.x-node.ref.x.back(),2) + pow(Pclose.y-node.ref.y.back(),2))/ref_res)+1;
	double N2 = round(sqrt( pow(Pfar.x-Pclose.x,2) + pow(Pfar.y-Pclose.y,2))/ref_res)+1;
	// Generate reference
	vector<double> Refx  = LinearSpacedVector(node.ref.x.back(),Pclose.x,N1);
	vector<double> Refxa = LinearSpacedVector(Pclose.x,Pfar.x,N2);
	vector<double> Refy  = LinearSpacedVector(node.ref.y.back(),Pclose.y,N1);
	vector<double> Refya = LinearSpacedVector(Pclose.y,Pfar.y,N2);
	ref.x.insert(ref.x.end(),Refx.begin(),Refx.end());
	ref.x.insert(ref.x.end(),Refxa.begin(),Refxa.end());
	ref.y.insert(ref.y.end(),Refy.begin(),Refy.end());
	ref.y.insert(ref.y.end(),Refya.begin(),Refya.end());
	
	// For debugging
	assert(ref.x.size()==ref.y.size());
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated goal reference."<<endl;}
	return ref;
};

// Generate a trapezoidal velocity profile
void generateVelocityProfile(MyReference& ref, const int& IDwp, const double& v0, const double& vmax, const vector<double>& goal, bool GB){
	// generateVelProfile(ref,v0,vmax,vend,IDwp,goal,GB)
	double vend = goal[3];
	// Slope shape configuration
	double a_acc = 1;	double a_dec = -1; 	double tmin = 1;  
	// Use first three point of reference to determine the resolution
	double Lpoints = sqrt( pow(ref.x[2]-ref.x[0],2) + pow(ref.y[2]-ref.y[0],2) ) ;
	double res = Lpoints/2; 		// ref_res of the reference path
	// Total length from the first waypoint until end of reference
	// cout<<"IDwp="<<IDwp<<", res="<<res<<endl;
	// double Lp = (ref.x.size()-IDwp-1)*res;
	double Lp = (ref.x.size()-IDwp)*res;
	// double Lp = (ref.x.size()-1)*res;
    if (!GB){
        double Dgoal = sqrt( pow(goal[0]-ref.x.back(),2) + pow(goal[1]-ref.y.back(),2));
        Lp = Lp + Dgoal;
	}
	
     
	// Check if the maximum coasting velocity can be reached for minimum time tmin
	double Daccel = (pow(vmax,2)- pow(v0,2))/(2*a_acc);
	double Dcoast = vmax*tmin;
	double Dbrake = (pow(vend,2)-pow(vmax,2))/(2*a_dec);
	double a2{-0.0252}, a1{1.2344}, a0{-0.5347};
	// double Dctrl = a2*pow(vmax,2) + a1*vmax +a0;
	double D_vmax_bool = (Daccel + Dcoast + Dbrake)<Lp;
    
    // Check what kind of velocity profile must be generated
	double Vcoast;
	if (vend>v0){
		Vcoast = vend;		 	// if end velocity greater than v0
	}else if (D_vmax_bool){
		Vcoast = vmax;			// If Vmax can be reached, Vcoast = Vmax
	}else{		// Else define as Dacc+Dcoast+Dbrake = D (solved in MATLAB for Vcoast)
        double D = Lp;
		// Vcoast calculation without compensation for tracking error
		double v1 =  ( sqrt(pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) - 2*D*pow(a_acc,2)*a_dec + pow(a_acc,2)*pow(vend,2) + 2*D*a_acc*pow(a_dec,2) - a_acc*a_dec*pow(v0,2) - a_acc*a_dec*pow(vend,2) + pow(a_dec,2)*pow(v0,2)) + a_acc*a_dec*tmin)/(a_acc - a_dec);       

		// Vcoast calculation with compensation for tracking error
		// double v = Vcoast;
		// double v1 = ( sqrt(pow(a_dec,2)*pow(v0,2) + pow(a_acc,2)*pow(vend,2) + pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) + 2*D*a_acc*pow(a_dec,2) - 2*D*pow(a_acc,2)*a_dec - 2*a0*a_acc*pow(a_dec,2) + 2*a0*pow(a_acc,2)*a_dec - a_acc*a_dec*pow(v0,2) - a_acc*a_dec*pow(vend,2) - 2*a1*a_acc*pow(a_dec,2)*v + 2*a1*pow(a_acc,2)*a_dec*v - 2*a2*a_acc*pow(a_dec,2)*pow(v,2) + 2*a2*pow(a_acc,2)*a_dec*pow(v,2)) + a_acc*a_dec*tmin)/(a_acc - a_dec);
		Vcoast = v1;
    }
    
	// Update profile distances with new Vcoast
	Daccel = ( pow(Vcoast,2)- pow(v0,2))/(2*a_acc);
	if(Daccel<0){		Daccel = 0; Vcoast = v0;    }
	double dv = Vcoast-vend;
	// Dctrl = a2*pow(dv,2) + a1*dv +a0;
	Dbrake = max(double(0), ( pow(vend,2)-pow(Vcoast,2))/(2*a_dec));
	Dcoast = max(double(0),Lp-Daccel-Dbrake);

	// Cubic polynomial interpolation
	double astart = 0;
	double vstart = v0+0.1;
	if (v0<=0.5){
		astart = 5;
	}
	ROS_WARN_STREAM_THROTTLE(1,"in ref: make acceleration sim dependent");
	vector<double> coef_acc = getCoefficients(Daccel,vstart,Vcoast,astart,0);
	vector<double> coef_brake = getCoefficients(Dbrake,Vcoast,goal[3],0,0);
	int Nacc = (Daccel/res)+1; double t_acc = 0; double dt_acc = double(1)/double(Nacc-1);
	int Nbrake = (Dbrake/res)+1; double t_brake = 0; double dt_brake = double(1)/double(Nbrake-1);
	for(int i = 0; i!=ref.x.size(); i++){
        double D = i*res;
        if(D<Daccel){
			// if (t_acc>1){ROS_WARN_STREAM("Constraining t_acc");}
			t_acc = t_acc*(t_acc<=1) + (t_acc>0);
			double v = getVelocity(vstart,coef_acc,t_acc);
			// cout<<"accelerate with t="<<t_acc<<", v="<<v<<endl;
			ref.v.push_back(v);
			t_acc += dt_acc;
        }else if (D<=(Daccel+Dcoast)){
            ref.v.push_back(Vcoast);
		}else{
			// if (t_brake>1){ROS_WARN_STREAM("Constraining t_brake");}
			t_brake = t_brake*(t_brake<=1) + (t_brake>0);
			double v = getVelocity(Vcoast,coef_brake,t_brake);
			// cout<<"braking with t="<<t_brake<<", v="<<v<<endl;
            ref.v.push_back(v);
			t_brake += dt_brake;
        }
    }
	// For debugging
	// double Dtotal = Daccel+Dbrake+Dcoast;
    // if (Dtotal<Lp){
	// 	ROS_WARN_STREAM("Velocity profile is too small!");
	// 	cout<<"IDwp="<<IDwp<<endl;
	// 	cout<<"Dacc = "<<Daccel<<". Dcoast = "<<Dcoast<<", Dbrake= "<<Dbrake<<endl;
	// 	cout<<"Lp="<<Lp<<", "<<Dtotal<<endl;
	// 	cout<<"Vcoast = "<<Vcoast<<endl;
	// }
	assert(ref.v.size()==ref.x.size());
}

// Print velocity profile in terminal
void showVelocityProfile(const MyReference& ref){
	cout<<endl<<"---Reference---"<<endl;
	cout<<"x = ["<<ref.x.front()<<", "<<ref.x.back()<<"]"<<endl;
	cout<<"y = ["<<ref.y.front()<<", "<<ref.y.back()<<"]"<<endl;
	cout<<"Velocity profile: "<<endl;
	auto it = ref.v.begin();
	for(it; it!=ref.v.end(); it++){
		cout<<*it<<", ";
	}
	cout<<endl<<endl;
}

// Get coefficients of cubic polynomial for velocity interpolation
vector<double> getCoefficients(const double& Sf, const double& v0, const double& vf, const double& a0, const double& af){
	double a = a0;
    double tf = 1;
    double c = (2*v0 - 2*vf + a0*tf + af*tf)/ pow(tf,3);
    double b = -(3*c*pow(tf,2) + a0 - af)/(2*tf);
	vector<double> coef{a,b,c};
	// cout<<"Coef: "<<a<<", "<<b<<", "<<c<<endl;
	return coef;
}

// Evaluate cubic polynomial representing the velocity profile
double getVelocity(const double& v0, const vector<double>& coef, const double& t){
	assert(coef.size()==3);
	double v = v0 + coef[0]*t + coef[1]*pow(t,2) + coef[2]*pow(t,3);
	return v;
}


// void generateVelocityProfile(	MyReference& ref, const double& _v0, const int& IDwp, const double& vmax, const double& vend){
// 	//// start generation of profile
// 	// Slope shape configuration
// 	double a_acc = 1;	  double a_dec = 1;	  double tmin = 1;   double v0 = _v0;
// 	// Total reference length
// 	double Ltotal = sqrt( pow(ref.x.back()-ref.x.front(),2) + pow(ref.y.back()-ref.y.front(),2) ) ;
// 	double res = Ltotal/(ref.x.size()-1); 		// ref_res of the reference path
// 	// Total shape length from the first waypoint until end of reference
// 	double Lp = sqrt( pow(ref.x.back()-ref.x[IDwp],2) +  pow(ref.y.back()-ref.y[IDwp],2) );
// 	int Np = (Lp/res)+1; int N0 = ref.x.size()-Np;
// 	// Check whether the maximum velocity can be reached for minimum time tmin
// 	double D_vmax_acc = (pow(vmax,2)- pow(v0,2))/(2*a_acc);
// 	double D_vmax_coast = vmax*tmin;
// 	double D_vmax_brake = (pow(vmax,2)- pow(vend,2))/(2*a_dec);
// 	// Boolean that states whether Vmax can be reached as coasting velocity
// 	bool D_vmax_bool = (D_vmax_acc + D_vmax_coast + D_vmax_brake)<Lp;
// 	double Vcoast;
// 	if (vend>v0){
// 		Vcoast = vend;		 	// if end velocity greater than v0
// 	}else if (D_vmax_bool){    	
// 		Vcoast = vmax;			// If Vmax can be reached, Vcoast = Vmax
// 	}else{ 						// Else define as Dacc+Dcoast+Dbrake = D (solved in MATLAB for Vcoast)
// 		Vcoast = ( sqrt( pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) + 2*Lp*pow(a_acc,2)*a_dec + pow(a_acc,2)*pow(vend,2) + 2*Lp*a_acc*pow(a_dec,2) + a_acc*a_dec*pow(v0,2) + a_acc*a_dec*pow(vend,2) + pow(a_dec,2)*pow(v0,2)  ) - a_acc*a_dec*tmin)/(a_acc + a_dec);
// 	}
// 	// Calculate profile distances etc
// 	double D_Vc_accel = ( pow(Vcoast,2)- pow(v0,2))/(2*a_acc); 			// Distance to accelerate to Vcoast
// 	if(D_Vc_accel<0){
// 		D_Vc_accel = 0; Vcoast = v0;
// 	}// Distance check
// 	double D_Vc_brake = max(double(0), ( pow(Vcoast,2) - pow(vend,2))/(2*a_dec));			// Braking distance
// 	//Dbrake = -0.0252* (Vcoast,2) + 1.2344*Vcoast -0.5347; 	// Additional braking distance to reduce controller overshoot
// 	double D_Vc_coast = max( double(0),Lp-D_Vc_accel-D_Vc_brake);					// Coasting distance

// 	///***************************************
// 	//Linear interpolation
// 	//****************************************/
// 	vector<double> vec_acc, vec_brake, vec_coast;
// 	if (D_Vc_accel!=0){
// 		vec_acc = LinearSpacedVector(v0,Vcoast,ceil(D_Vc_accel/res)+1);
// 	}
// 	if(D_Vc_brake!=0){
// 		vec_brake = LinearSpacedVector(Vcoast,vend,floor(D_Vc_brake/res)+1);
// 	}
// 	if(D_Vc_coast!=0){
// 		while(vec_coast.size()!=(Np-vec_acc.size()-vec_brake.size()))
// 		{  	vec_coast.push_back(Vcoast);	}
// 	}
// 	ref.v.clear();
// 	ref.v.insert(ref.v.end(),vec_acc.begin(),vec_acc.end());
// 	ref.v.insert(ref.v.end(),vec_coast.begin(),vec_coast.end());
// 	ref.v.insert(ref.v.end(),vec_brake.begin(),vec_brake.end());
// 	// Append front with initial velocity
// 	while( ref.v.size()<ref.x.size()){
// 		ref.v.insert(ref.v.begin(), v0);
// 	}
// 	assert(ref.v.size()==ref.x.size());

// 	if(debug_reference)
// 	{
// 		cout<<"IDwp="<<IDwp<<",\t Daccel="<<D_Vc_accel<<"\tDcoast="<<D_Vc_coast<<"\tDbrake="<<D_Vc_brake<<"\t Lp="<<Lp<<endl;
// 		cout<<"Reference path:"<<endl;
// 		for(vector<double>::iterator it = ref.x.begin(); it!=ref.x.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 		cout<<endl;
// 		for(vector<double>::iterator it = ref.y.begin(); it!=ref.y.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 		cout<<endl<<"Reference velocity profile:"<<endl;
// 		// For debugging
// 		for(vector<double>::iterator it = ref.v.begin(); it!=ref.v.end(); ++it){
// 			cout<<*it<<"-";
// 		}
// 	}	
	
// 	if(debug_mode){
// 		cout<<"Generated velocity profile. "<<endl;
// 	}
// 	assert(ref.v.size()==ref.x.size());	
// }
