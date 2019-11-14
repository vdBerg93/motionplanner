#ifndef CONTROL_H
#define CONTROL_H

class Observer{
    private:
        ros::Publisher* ptrPub;
        bool updateControls();
        double a_cmd, d_cmd;
        vector<double> carPose;
        car_msgs::MotionResponse path;
        prius_msgs::Control genMoveMsg();
        prius_msgs::Control genStaticMsg();
    public:
        Observer(ros::Publisher* pub): ptrPub(pub){

        }   
        void callbackState(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void callbackMotion(const car_msgs::MotionResponse& msg);
        bool publishControls();


};

#endif