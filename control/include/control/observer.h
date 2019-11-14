#ifndef CONTROL_H
#define CONTROL_H

class Observer{
    private:
        ros::Publisher* ptrPub;
        bool updateControls();
        double a_cmd, d_cmd, v_cmd;
        vector<double> carState;
        car_msgs::MotionResponse path;
        prius_msgs::Control genMoveMsg();
        prius_msgs::Control genStaticMsg();
    public:
        Observer(ros::Publisher* pub): ptrPub(pub){

        }   
        void callbackState(const car_msgs::State& msg);
        void callbackMotion(const car_msgs::MotionResponse& msg);
        bool publishControls();


};

#endif