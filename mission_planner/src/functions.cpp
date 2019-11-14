double convertQuaternionToEuler(geometry_msgs::Quaternion input);
double getGoalVelocity();
double getSpeedLimit();
bool waitForConfirmation();
// void sendMotionRequest(const ros::Publisher* ptrPubMP, const vector<double>& goal, const double& Vmax);

struct MsgManager{
    MsgManager(): confirmed(0), goalReceived(0){}
    vector<double> carPose;
    double Vgoal, Vmax;
    vector<double> goalW, goalC;
    bool confirmed, goalReceived;
    ros::Publisher* ptrPubMP;
    void stateCallback(const car_msgs::State& input);
    void goalCallback(geometry_msgs::PoseStamped input);
    void motionCallback(car_msgs::MotionResponse resp);
    void sendMotionRequest();
    void updateGoalCar();
};

void MsgManager::stateCallback(const car_msgs::State& input){
    // vector<double> state = {input.pose.pose.position.x,input.pose.pose.position.y,convertQuaternionToEuler(input.pose.pose.orientation)};
    carPose=input.state;
}

void MsgManager::goalCallback(geometry_msgs::PoseStamped input){
    cout<<"Goal pose received! (x,y,theta,vel)"<<endl;
    Vgoal = getGoalVelocity();
    goalW = {input.pose.position.x,input.pose.position.y,convertQuaternionToEuler(input.pose.orientation), Vgoal};
    
    ros::param::get("max_velocity",Vmax);
    cout<<"Maximum velocity = "<<Vmax<<" m/s"<<endl;
    assert(Vmax<=8.33); // Speed limit
    goalReceived =true;
    // Vmax = getSpeedLimit();
    // sendMotionRequest(ptrPubMP, goalC, Vmax);
    // confirmed = waitForConfirmation();
}

void MsgManager::motionCallback(car_msgs::MotionResponse resp){
    
}

void MsgManager::updateGoalCar(){
    goalC = {goalW[0]-carPose[0],goalW[1]-carPose[1],goalW[2]-carPose[2],goalW[3]};
    
}

void MsgManager::sendMotionRequest(){
    car_msgs::MotionRequest req;
    updateGoalCar();
    
    for(auto it = goalC.begin(); it!=goalC.end(); it++){
        req.goal.push_back(*it);
    }
    req.vmax = Vmax;
    req.bend = false;
    cout<<"Goal (map): "<<"["<<goalW[0]<<", "<<goalW[1]<<", "<<goalW[2]<<", "<<goalW[3]<<"]"<<endl;
    cout<<"Goal (car): "<<"["<<goalC[0]<<", "<<goalC[1]<<", "<<goalC[2]<<", "<<goalC[3]<<"]"<<endl;

    ptrPubMP->publish(req);
}

bool waitForConfirmation(){
    string userInput;
    string Y = "yes";
    string N = "no";
    while (1){
        cout<<"Start performing the motion plan? (yes/no) "<<endl;
        cin >> userInput;
        if (userInput==Y){
            return true;
        }
        if (userInput==N){
            return false;
        }
    }
}

double getGoalVelocity(){
    double Vgoal = 0;
    return Vgoal;
}
double getSpeedLimit(){
    double Vmax = 100000;
    double Vlimit = 30/3.6;
    while (Vmax>Vlimit){
        cout<<"Please enter the speed limit (max "<<Vlimit<<" m/s): ";
        cin >> Vmax;
    }
    return Vmax;
}

double convertQuaternionToEuler(geometry_msgs::Quaternion input){
    // Return the yaw angle from the input message
    double siny_cosp = 2 * (input.w * input.z + input.x * input.y);
    double cosy_cosp = 1 - 2 * (input.y * input.y + input.z * input.z);
    return atan2(siny_cosp, cosy_cosp);
}
