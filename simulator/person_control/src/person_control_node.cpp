//#######################################################
//## Control of pedestrian #######
//#######################################################
// using namespace std;
#include <ros/ros.h>
#include <pedsim_msgs/AgentStates.h>
#include <geometry_msgs/PoseStamped.h>

struct Observer{
    ros::Publisher* ptrPub;
    Observer(){};
    void callback(const pedsim_msgs::AgentStates& msgIn);
};

//#### MAIN FUNCTION ####################################
int main( int argc, char** argv ){	
	// Initialize node
    ros::init(argc, argv, "person_control_node");
	ros::NodeHandle nh;

    // Communication object
    Observer ObserveObject;

	// // Create a ROS subscriber and publisher
	ros::Subscriber sub = nh.subscribe ("/pedsim_simulator/simulated_agents", 1, &Observer::callback,&ObserveObject);
	ros::Publisher pub =nh.advertise<geometry_msgs::PoseStamped>("/ped_link_1", 100);

    ObserveObject.ptrPub = &pub;

	ros::Rate r(40);
	while( ros::ok() ){
		ros::spinOnce();
		r.sleep();
	}
}

void Observer::callback(const pedsim_msgs::AgentStates& msgIn){
    geometry_msgs::PoseStamped msgOut;
    msgOut.header = msgIn.header;
    msgOut.pose.position = msgIn.agent_states.front().pose.position;
    // Fix orientation
    msgOut.pose.orientation.w = 0;
    msgOut.pose.orientation.z = 0;
    msgOut.pose.orientation.y = 0;
    msgOut.pose.orientation.x = 0;
    ptrPub->publish(msgOut);
}