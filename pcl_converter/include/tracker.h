struct Tracker{
    int ID;
	vector<double> xpos;
	vector<double> ypos;
	vector<double> xvel;
	vector<double> yvel;
	int countLost;
	Tracker(int id, double x0, double y0) : ID(id), countLost(0){
        xpos.push_back(x0), ypos.push_back(y0); // Initialize position
		xvel.push_back(0); yvel.push_back(0);	// Initialize velocity
	}
	void update(double x, double y){
		xpos.push_back(x);
		ypos.push_back(y);
        double vx = xpos.back()-*(xpos.end()-1)/0.05;
        double vy = ypos.back()-*(ypos.end()-1)/0.05;
        xvel.push_back(vx); yvel.push_back(vy);
	}
};
