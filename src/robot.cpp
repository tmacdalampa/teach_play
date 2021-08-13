#include "teach_play/robot.h"

using namespace std;



Robot::Robot(ros::NodeHandle *nh)
{	
	if( !nh->getParam("/enc_resolution", _enc_resolution))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/gear_ratios", _gear_ratios))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/zero_points", _zero_points))
    	ROS_ERROR("Failed to get parameter from server.");

    joint_state_pub = nh->advertise<std_msgs::Int64>("/number_count", 10);

    reset_service = nh->advertiseService("/reset_counter", &Robot::callback_reset_counter, this);
}

Robot::~Robot()
{

}

void Robot::JointStatesPublisher()
{
	std_msgs::Int64 new_msgs;
	new_msgs.data = 1;
	//cout << new_msgs.data << endl;
	joint_state_pub.publish(new_msgs);
}

bool Robot::callback_reset_counter(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	res.success = true;
	res.message = "hello";
	cout << res.message << endl;
    return true;
}