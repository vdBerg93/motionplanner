
/* --------------------------------------
	REFERENCE GENERATION
---------------------------------------*/
#include "rrt/rrtplanner.h"
#include "ros/ros.h"

MyReference getReference(geometry_msgs::Point sample, Node node, signed int dir){
	MyReference ref;
	double L = sqrt( pow(sample.x-node.ref.x.back(),2) + pow(sample.y-node.ref.y.back(),2) );
	int N = round(L/ref_res)+1;
	//cout<<"res="<<ref_res<<" L="<<L<<" n="<<N<<endl;
	ref.x = LinearSpacedVector(node.ref.x.back(),sample.x,N);
	ref.y = LinearSpacedVector(node.ref.y.back(),sample.y,N);
	ref.dir = dir;
	assert(ref.x.size()==ref.y.size());
	assert(ref.x.size()>=3);
	if(debug_mode){cout<<"Generated reference."<<endl;}
	return ref;
};

MyReference getGoalReference(const Vehicle& veh, Node node, vector<double> goalPose){;
	double Dextend = ctrl_dla+10;//1.2;
	double Dalign = 0.1;
	
	MyReference ref;
	//double ref_res;	ros::param::get("/planner/ref_res",ref_res);
	//double ref_res = max(node.state[4]*ref_interval,ref_mindist);

	// Alignment
	geometry_msgs::Point P1, P2, Pclose, Pfar;
	P1.x = goalPose[0]+Dalign*cos(goalPose[2]); P1.y = goalPose[1]+Dalign*sin(goalPose[2]);
	P2.x = goalPose[0]-Dalign*cos(goalPose[2]); P2.y = goalPose[1]-Dalign*sin(goalPose[2]);
	
	// Select closest point
	if( sqrt( pow(P1.x-node.ref.x.back(),2) + pow(P1.y-node.ref.y.back(),2)) < sqrt( pow(P2.x-node.ref.x.back(),2) + pow(P2.y-node.ref.y.back(),2))){
		Pclose = P1; Pfar = P2; 
	}else{
		Pclose = P2; Pfar = P1;
	}
	// Extend to account for lookahead distance
	Pfar.x += Dextend*cos(goalPose[2]);
	Pfar.y += Dextend*sin(goalPose[2]);
	assert(Pfar.y==Pclose.y);
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

void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend){
	//// start generation of profile
	// Slope shape configuration
	double a_acc = 1;	  double a_dec = 1;	  double tmin = 1;   double v0 = node.state[4];
	// Total reference length
	double Ltotal = sqrt( pow(ref.x.back()-ref.x.front(),2) + pow(ref.y.back()-ref.y.front(),2) ) ;
	double res = Ltotal/(ref.x.size()-1); 		// ref_res of the reference path
	// Total shape length from the first waypoint until end of reference
	double Lp = sqrt( pow(ref.x.back()-ref.x[IDwp],2) +  pow(ref.y.back()-ref.y[IDwp],2) );
	int Np = (Lp/res)+1; int N0 = ref.x.size()-Np;
	// Check whether the maximum velocity can be reached for minimum time tmin
	double D_vmax_acc = (pow(vmax,2)- pow(v0,2))/(2*a_acc);
	double D_vmax_coast = vmax*tmin;
	double D_vmax_brake = (pow(vmax,2)- pow(vend,2))/(2*a_dec);
	// Boolean that states whether Vmax can be reached as coasting velocity
	bool D_vmax_bool = (D_vmax_acc + D_vmax_coast + D_vmax_brake)<Lp;
	double Vcoast;
	if (vend>v0){
		Vcoast = vend;		 	// if end velocity greater than v0
	}else if (D_vmax_bool){    	
		Vcoast = vmax;			// If Vmax can be reached, Vcoast = Vmax
	}else{ 						// Else define as Dacc+Dcoast+Dbrake = D (solved in MATLAB for Vcoast)
		Vcoast = ( sqrt( pow(a_acc,2)*pow(a_dec,2)*pow(tmin,2) + 2*Lp*pow(a_acc,2)*a_dec + pow(a_acc,2)*pow(vend,2) + 2*Lp*a_acc*pow(a_dec,2) + a_acc*a_dec*pow(v0,2) + a_acc*a_dec*pow(vend,2) + pow(a_dec,2)*pow(v0,2)  ) - a_acc*a_dec*tmin)/(a_acc + a_dec);
	}
	// Calculate profile distances etc
	double D_Vc_accel = ( pow(Vcoast,2)- pow(v0,2))/(2*a_acc); 			// Distance to accelerate to Vcoast
	if(D_Vc_accel<0){
		D_Vc_accel = 0; Vcoast = v0;
	}// Distance check
	double D_Vc_brake = max(double(0), ( pow(Vcoast,2) - pow(vend,2))/(2*a_dec));			// Braking distance
	//Dbrake = -0.0252* (Vcoast,2) + 1.2344*Vcoast -0.5347; 	// Additional braking distance to reduce controller overshoot
	double D_Vc_coast = max( double(0),Lp-D_Vc_accel-D_Vc_brake);					// Coasting distance

	// Loop through points and generate profile
	//L = 0;
	///***************************************
	//Cubic Bezier curve interpolation
	//***************************************/
	// ROS_WARN_STREAM("Still an error in this part!");
	// // Acceleration interpolation
	// vector<> t_a = LinearSpacedVector(0,1,(Daccel/res)+1);
	// vector<> cp_a{ v0, v0, Vcoast, Vcoast};
	// Vacc = bezierCurveInterpolation(cp_a,t_a);
	// // Deceleration interpolation
	// vector<> t_d = LinearSpacedVector(0,1,(Dbrake/res)+1);
	// vector<> cp_d{ Vcoast, Vcoast, vend, vend};
	// vector<> Vdec = bezierCurveInterpolation(cp_d,t_d);

	///***************************************
	//Linear interpolation
	//****************************************/
	vector<double> vec_acc, vec_brake, vec_coast;
	if (D_Vc_accel!=0){
		vec_acc = LinearSpacedVector(v0,Vcoast,ceil(D_Vc_accel/res)+1);
	}
	if(D_Vc_brake!=0){
		vec_brake = LinearSpacedVector(Vcoast,vend,floor(D_Vc_brake/res)+1);
	}
	if(D_Vc_coast!=0){
		while(vec_coast.size()!=(Np-vec_acc.size()-vec_brake.size()))
		{  	vec_coast.push_back(Vcoast);	}
	}

	ref.v.insert(ref.v.end(),vec_acc.begin(),vec_acc.end());
	ref.v.insert(ref.v.end(),vec_coast.begin(),vec_coast.end());
	ref.v.insert(ref.v.end(),vec_brake.begin(),vec_brake.end());
	// Append front with initial velocity
	while( ref.v.size()<ref.x.size()){
		ref.v.insert(ref.v.begin(), v0);
	}
	assert(ref.v.size()==ref.x.size());
	// FOR DEBUGGING ONLY
	// MaxAccel = 1.5*Vcoast/(1.5*Daccel-1.5*c1*Daccel);
	// MaxDecel = 1.5*-Vcoast/(1.5*Daccel-1.5*c1*Daccel);

	// SHORTENING (NOT REQUIRED?)
	// while (((vec_acc.size() + vec_coast.size()+vec_brake.size()) > Np)&&(!vec_coast.empty)){
	// 	vec_coast.pop_back();
	// }
	// while( ref.v.size()!=ref.x.size()){
	// 	if(ref.v.size()<Np){
	// 		// Append untill brake vector is empty
	// 		if (!vec_brake.empty()){
	// 			ref.v.push_back(vec_brake.front());		vec_brake.erase(vec_brake.begin()); continue;
	// 		// Append untill coast vector is empty
	// 		}else if (!vec_coast.empty()){
	// 			ref.v.insert(ref.v.begin(), vec_coast.back()); vec_coast.pop_back(); continue;
	// 		// Append until accelerate vector is empty
	// 		}else if (!vec_acc.empty()){
	// 			ref.v.insert(ref.v.begin(), vec_acc.back()); vec_acc.pop_back(); continue;
	// 		// Append the rest with v0 (not used)s
	// 		}else{
	// 			ref.v.insert(ref.v.begin(), v0); vec_acc.pop_back(); continue;
	// 		}	
	// 	}else{
	// 		ref.v.insert(ref.v.begin(), v0);
	// 	}
	// }
	if(debug_reference)
	{
		cout<<"IDwp="<<IDwp<<",\t Daccel="<<D_Vc_accel<<"\tDcoast="<<D_Vc_coast<<"\tDbrake="<<D_Vc_brake<<"\t Lp="<<Lp<<endl;
		cout<<"Reference path:"<<endl;
		for(vector<double>::iterator it = ref.x.begin(); it!=ref.x.end(); ++it){
			cout<<*it<<"-";
		}
		cout<<endl;
		for(vector<double>::iterator it = ref.y.begin(); it!=ref.y.end(); ++it){
			cout<<*it<<"-";
		}
		cout<<endl<<"Reference velocity profile:"<<endl;
		// For debugging
		for(vector<double>::iterator it = ref.v.begin(); it!=ref.v.end(); ++it){
			cout<<*it<<"-";
		}
	}	
	
	if(debug_mode){
		cout<<"Generated velocity profile. "<<endl;
	}
	assert(ref.v.size()==ref.x.size());	
}


//while( length(ref.v)~=length(ref.x) )
    //if length(ref.v)<Np
      //  if ~isempty(vec_brake)
        //    ref.v = [ref.v,vec_brake(1)]; vec_brake(1)=[]; continue;
        //elseif ~isempty(vec_coast)
        //    ref.v = [vec_coast(end),ref.v]; vec_coast(end)=[]; continue;
       // elseif ~isempty(vec_acc)
       //     ref.v = [vec_acc(end),ref.v]; vec_acc(end)=[]; continue;
//         else
//             ref.v = [v0,ref.v]; continue;
//         end
//     else
//         ref.v = [v0,ref.v]; continue;
//     end
// end

// void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend){
// 	//ROS_ERROR_STREAM("FIX VEL PROFILE GENERAATION!");
// 	//ROS_WARN_STREAM("vmax="<<vmax<<" vend="<<vend);
// 	assert(ref.v.size()==0);
// 	// Slope shape configuration
// 	double a_acc = 0.75;	double a_dec = 0.75;	double tmin = 1; double v0 = node.state[4];
// 	// Total reference length
// 	double Lref = sqrt( pow(ref.x.back()-ref.x[0],2) + pow(ref.y.back()-ref.y[0],2) );
// 	// Total shape length from the first waypoint until end of reference
// 	double D = sqrt( pow(ref.x.back()-ref.x[IDwp],2) + pow(ref.y.back()-ref.y[IDwp],2) );
// 	// Check whether the maximum velocity can be reached for minimum time tmin
// 	double Da = (pow(vmax,2)-pow(v0,2))/(2*a_acc); 		// acceleration
// 	double Dc = vmax*tmin; 								// coasting
// 	double Db = (pow(vmax,2)-pow(vend,2))/(2*a_dec);	// braking
// 	//Db += -0.0252*pow(vmax,2) + 1.2344*vmax -0.5347; // compensate for overshoot
// 	// Boolean that states whether Vmax can be reached as coasting velocity
// 	bool BOOL = (Da+Dc+Db)<D;	double Vcoast;
// 	// If Vmax can be reached, Vcoast = Vmax
// 	if(BOOL){ 	
// 		Vcoast = vmax;
// 	}// Else, set Vcoast such that a minimum coasting time tmin is achieved
// 	else{		
// 			ROS_WARN_STREAM("in vel profile: CHeck this for validity!"); // implemented vend check if eq. is correct
// 			Vcoast = -(a_dec*(a_acc*tmin - sqrt((a_dec*pow(a_acc,2)*pow(tmin,2) + 2*D*pow(a_acc,2) + a_acc*pow(v0,2) + 2*a_dec*D*a_acc + a_dec*pow(v0,2) )/a_dec)))/(a_acc + a_dec);
// 	}	
// 	// Calculate profile distances etc
// 	double Daccel = (pow(Vcoast,2)-pow(v0,2))/(2*a_acc); 			// Distance to accelerate to Vcoast
// 	if(Daccel<0){Daccel = 0; Vcoast = v0;}; 						// Distance check
// 	double Dbrake = (pow(Vcoast,2)-pow(vend,2))/(2*a_dec);			// Braking distance
// 	//Dbrake = -0.0252*pow(Vcoast,2) + 1.2344*Vcoast -0.5347; 	// Additional braking distance to reduce controller overshoot
// 	double Dcoast = max(double(0),D-Daccel-Dbrake);					// Coasting distance
// 	double res = Lref/ref.x.size(); 								// ref_res of the reference path
// 	// Control points of the velocity profile
// 	double p3 = Lref-Dbrake;	// Start braking
// 	double p2 = p3-Dcoast; 		// Start coasting
// 	double p1 = p2-Daccel;	 	// Start accelerating
// 	//p1 = std::max(double(0),p1-2*res);
// 	// Loop through points and generate profile
// 	double L = 0;

// 	while(ref.v.size()!=ref.x.size()){
// 		L = L+res;
// 		if (L<p1)
// 		{
// 			// Before ramp up point, constant initial velocity.
// 			// Value is not really relevant since this part is skipped due to the lookahead distance
// 			ref.v.push_back(v0);	continue;
// 		}
// 		if (Daccel!=0){ // If an acceleration part exists, use this equation
// 			ref.v.push_back(v0 + ((p1<=L)&&(L<=p2))*(((L-p1)/Daccel)*(Vcoast-v0)) + (p2<L)*(Vcoast-v0) + (p3<L)*((L-p3)/Dbrake)*(-(Vcoast-vend)));
// 		}
// 		else if ((Daccel==0)&&(Dcoast>0)){ 	// If no acceleration but coasting for time t, use this equation
// 			//Dcoast = D-Dbrake;
// 			if(Dbrake>0){
// 				ref.v.push_back( v0 + (L>=p3)*((L-p3)/Dbrake)*(-v0) );
// 			}else{
// 				ref.v.push_back( Vcoast);
// 			}
// 		}
// 		else{ // Only braking
// 			ref.v.push_back(v0 - (Vcoast-vend)*L/Dbrake );
// 		}
// 	}
// //	cout<<"L(//)="<<(L/Lref)<<endl;

// 	if(debug_reference)ref length="<<ref.x.size();
// 		cout<<"IDwp="<<IDwp<<",\t Daccel="<<Daccel<<"\tDcoast="<<Dcoast<<"\tDbrake="<<Dbrake<<"\t D="<<D<<endl;
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
// 		cout<<"lengths: "<<"D="<<D<<" Daccel="<<Daccel<<" Dbrake="<<Dbrake<<" Dcoast="<<Dcoast<<endl;
// 	}{		
// 		cout<<"
// 	assert(ref.v.size()==ref.x.size());
// 	if(debug_mode){
// 		cout<<"Generated velocity profile. "<<endl;
// 		}
// //	return;
// }


// void generateVelocityProfile(	MyReference& ref, const Node& node, const int& IDwp, const double& vmax, const double& vend){
// 	// Slope shape configuration
// 	double a_acc = 0.75;	double a_dec = 0.75;	double tmin = 1; double v0 = node.state[4];
// 	// Total reference length
// 	double Lref = sqrt( pow(ref.x.back()-ref.x[0],2) + pow(ref.y.back()-ref.y[0],2) );
// 	// Total shape length from the first waypoint until end of reference
// 	double D = sqrt( pow(ref.x.back()-ref.x[IDwp],2) + pow(ref.y.back()-ref.y[IDwp],2) );
// 	// Check whether the maximum velocity can be reached for minimum time tmin
// 	double Da = (pow(vmax,2)-pow(v0,2))/(2*a_acc);
// 	double Dc = vmax*tmin;
// 	double Db = (pow(vmax,2)-pow(v0,2))/(2*a_dec) -0.0252*pow(vmax,2) + 1.2344*vmax -0.5347;
// 	bool BOOL = (Da+Dc+Db)<D;
	
// 	double Vcoast;
// 	if(BOOL){ 	// If Vcoast can be reached, Vcoast = Vmax
// 		Vcoast = vmax;
// 	}else{		// Else, Vcoast = ?
// 			Vcoast = -(a_dec*(a_acc*tmin - sqrt((a_dec*pow(a_acc,2)*pow(tmin,2) + 2*D*pow(a_acc,2) + a_acc*pow(v0,2) + 2*a_dec*D*a_acc + a_dec*pow(v0,2) )/a_dec)))/(a_acc + a_dec);
// 	}	

// 	ROS_INFO_STREAM("Vcoast="<<Vcoast);
// 	assert(!isnan(Vcoast));

// 	double Daccel = (pow(Vcoast,2)-pow(v0,2))/(2*a_acc); 		
// 	if(Daccel<0){Daccel = 0; Vcoast = v0;}; 				
// 	double Dbrake = (pow(Vcoast,2)-pow(v0,2))/(2*a_dec);
// 	Dbrake = Dbrake -0.0252*pow(Vcoast,2) + 1.2344*Vcoast -0.5347;
// 	double Dcoast = max(double(0),D-Daccel-Dbrake);
// 	ROS_WARN_STREAM("D="<<D<<" Daccel="<<Daccel<<" Dbrake="<<Dbrake<<" Dcoast="<<Dcoast);
// 	// Ramp up/down generation
// 	double res = Lref/ref.x.size();
// 	/***************************************
// 	Cubic Bezier curve interpolation
// 	***************************************/
// 	// ROS_WARN_STREAM("Still an error in this part!");
// 	// // Acceleration interpolation
// 	// vector<double> t_a = LinearSpacedVector(0,1,(Daccel/res)+1);
// 	// vector<double> cp_a{ v0, v0, Vcoast, Vcoast};
// 	// Vacc = bezierCurveInterpolation(cp_a,t_a);
// 	// // Deceleration interpolation
// 	// vector<double> t_d = LinearSpacedVector(0,1,(Dbrake/res)+1);
// 	// vector<double> cp_d{ Vcoast, Vcoast, vend, vend};
// 	// vector<double> Vdec = bezierCurveInterpolation(cp_d,t_d);

// 	/***************************************
// 	Linear interpolation
// 	****************************************/
// 	vector<double> Vacc;
// 	if(Daccel!=0)
// 	{
// 		vector<double> Vacc = LinearSpacedVector(v0,Vcoast,(Daccel/res)+1);
// 	}
// 	vector<double> Vdec = LinearSpacedVector(Vcoast,vend,(Dbrake/res)+1);
// 	assert(Dbrake<1000);
// 	assert(Vdec.size()<1000);
	
// 	// FOR DEBUGGING ONLY
// 	//double MaxAccel = 1.5*Vcoast/(1.5*Daccel-1.5*c1*Daccel);
// 	//double MaxDecel = 1.5*-Vcoast/(1.5*Daccel-1.5*c1*Daccel);

// 	//assert(Vacc.size()>0); assert(Vdec.size()>0);
// 	int ID_start = ref.x.size()-((D/res));
// 	int ID_accelerate = ref.x.size()-((D/res));
// 	int ID_coast = ID_accelerate+Vacc.size();
// 	int ID_brake = ref.x.size()-Vdec.size();
	
// 	 ROS_WARN_STREAM("IDa="<<ID_accelerate<<" IDc="<<ID_coast<<" IDb="<<ID_brake);
// 	 ROS_WARN_STREAM("sztotal="<<ref.x.size()<<" sza="<<Vacc.size()<<" szb="<<Vdec.size());

// 	cout<<endl;
// 	while(ref.v.size()<ID_start){
// 		ref.v.push_back(v0);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("Finished start part...");
// 	while(Vacc.size()!=0){
// 		ref.v.push_back(Vacc.front());
// 		Vacc.erase(Vacc.begin());
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("Finished acceleration");
// 	while((ref.v.size()-ID_brake)>0){
// 		ref.v.push_back(Vcoast);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	//ROS_INFO_STREAM("FInished coasting");
// 	for(int i = 0; i<=Vdec.size(); i++){
// 		ref.v.push_back(Vdec[i]);
// 		cout<<ref.v.back()<<", ";
// 	}
// 	// while(Vdec.size()!=0){
// 	// 	ref.v.push_back(Vdec.front());
// 	// 	Vdec.erase(Vdec.begin());
// 	// 	cout<<ref.v.back()<<', ';
// 	// }
// 	ROS_INFO_STREAM("Finished braking");

// 	// // Loop through points and generate profile
// 	// for(int i = 0; i<ref.x.size(); i++){
// 	// 	//ROS_INFO_STREAM("Loop at i="<<i);
// 	// 	if(((i<(ID_accelerate))&&(Daccel!=0))){
// 	// 		//ROS_INFO_STREAM("Skipping...");	
// 	// 		ref.v.push_back(0); continue;
// 	// 	}
// 	// 	//else if(L<=p2){
// 	// 	else if((i<(ID_coast))&&(Dcoast!=0))
// 	// 	{
// 	// 		ref.v.push_back(Vacc[0]);
// 	// 		Vacc.erase(Vacc.begin());
// 	// 		//ROS_INFO_STREAM("Accelerating...");
// 	// 	}
// 	// 	//else if(L<p3){
// 	// 	else if(i<=(ID_brake)){
// 	// 		ref.v.push_back(Vcoast);
// 	// 		//ROS_INFO_STREAM("Coasting...");
// 	// 	}
// 	// 	else{
// 	// 		ref.v.push_back(Vdec[0]);
// 	// 		Vdec.erase(Vdec.begin());
// 	// 		//ROS_INFO_STREAM("Decellerating...");
// 	// 	}
		
// 	// }

// 	// For debugging
// 	// for(int i = 0; i<=ref.v.size();i++){
// 	// 	std::cout<<ref.v[i]<<" ,";
// 	// }
// 	ROS_ERROR_STREAM("Error inside this function!");
// 	//ROS_WARN_STREAM("Vacc sz="<<Vacc.size()<<" Vdec sz="<<Vdec.size());
// 	assert(ref.v.size()==ref.x.size());
// 	ROS_INFO_STREAM("TEST");
// 	return;
// }

//  */