#ifndef CONTROL_H
#define CONTROL_H

class Observer{
    private:
        ros::Publisher* ptrPub;
        ros::Publisher* ptrPubError1;
        ros::Publisher* ptrPubError2;
        ros::Publisher* ptrPubError3;
        ros::Publisher* ptrPubError4;
        bool updateControls();
        double a_cmd, d_cmd, v_cmd;
        vector<double> carState;
        car_msgs::MotionResponse path;
        prius_msgs::Control genMoveMsg();
        prius_msgs::Control genStaticMsg();
    public:
        Observer(ros::Publisher* pub, ros::Publisher* pubError1, ros::Publisher* pubError2, ros::Publisher* pubError3, ros::Publisher* pubError4): 
        ptrPub(pub), ptrPubError1(pubError1), ptrPubError2(pubError2), ptrPubError3(pubError3), ptrPubError4(pubError4){}   
        void callbackState(const car_msgs::State& msg);
        void callbackMotion(const car_msgs::MotionResponse& msg);
        bool publishControls();
};

#endif