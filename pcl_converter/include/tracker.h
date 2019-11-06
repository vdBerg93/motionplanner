// struct Tracker{
// 	vector<double> xpos;
// 	vector<double> ypos;
// 	vector<double> xvel;
// 	vector<double> yvel;
// 	int countLost;
// 	Tracker(const double& x0, const double& y0): countLost(0){
//         xpos.push_back(x0), ypos.push_back(y0); // Initialize position
// 		xvel.push_back(0); yvel.push_back(0);	// Initialize velocity
// 	}
// 	void update(const double& x, const double& y, car_msgs::Obstacle2D& obs){
// 		xpos.push_back(x);
// 		ypos.push_back(y);
// 		double dx = xpos.back()-xpos[xpos.size()-2];
// 		double dy = ypos.back()-ypos[ypos.size()-2];
//         xvel.push_back(dx/0.05); yvel.push_back(dy/0.05);
// 		// showall();
// 		updateOBB(obs);
// 	}
// 	void updateOBB(car_msgs::Obstacle2D& obs){
// 		obs.vel.linear.x = xvel.back();
// 		obs.vel.linear.y = yvel.back();
// 		cout<<"Obs_state = ["<<obs.obb.center.x<<", "<<obs.obb.center.y<<", "<<obs.vel.linear.x<<", "<<obs.vel.linear.y<<" ]"<<endl;
// 	}
// };

struct Tracker{
	KalmanFilter kf;
	int m, n;
	int countLost;
	Tracker(const double& xinit, const double& yinit): countLost(0), n(4), m(2){
		Eigen::VectorXd x0(n);		// Empty state vector [1xn]
		x0 << xinit, yinit, 0, 0;	// Fill vector with initial state
		kf.init(0,x0);				// Initialize Kalman filter
	}
	void update(const double& xm, const double& ym, car_msgs::Obstacle2D& obs){
		if (DEBUG) {cout<<"Updating KF"<<endl;}
		Eigen::VectorXd y(m);		// Empty measurement vector [1xm]
		y << xm, ym;				// Fill vector with measurements
		kf.update(y);				// Update Kalman filter
		updateOBB(obs);
	}
	void updateOBB(car_msgs::Obstacle2D& obs){
		if(DEBUG){cout<<"Updating OBB"<<endl;}
		Eigen::VectorXd X = kf.state();
		obs.vel.linear.x = X(2);
		obs.vel.linear.y = X(3);
		cout<<"KF_state="<<kf.state().transpose()<<endl;
		cout<<"Obs_state = ["<<obs.obb.center.x<<", "<<obs.obb.center.y<<", "<<obs.vel.linear.x<<", "<<obs.vel.linear.y<<" ]"<<endl;
	}
};