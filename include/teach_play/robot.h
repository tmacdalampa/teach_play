//include ros library
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>


#include "teach_play/hardware_transmission_interface.h"
#include <array>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <string>

#define JNT_NUM 6
#define PI 3.14157
#define DEG_PER_REV 360
#define DEG2RAD 0.0017453

using namespace std;

class HwTmIntf;


class Robot
{
public:

	ros::Publisher joint_state_pub;
	ros::Publisher robot_pose_pub;
	ros::ServiceServer reset_service;

	Robot(ros::NodeHandle *nh);
	~Robot();
	void JointStatesPublisher();
	void RobotPosePublisher();
	bool callback_reset_counter(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:


	vector<int> _gear_ratios;
	vector<int> _zero_points;
	int _enc_resolution;
	int _jnt_num;
};

/*
struct DHtable
{
	array<double, MAX_MOTOR> a;
	array<double, MAX_MOTOR> alpha;
	array<double, MAX_MOTOR> d;

	void getT(Maxrix4d, &out_T, double JointDeg, int id)
	{
		double A, D, sa, ca, cs, ss;

		id = id -1;

		A = a[id];
		ss = sin( axisDeg * DEG2RAD);
		cs = cos( axisDeg * DEG2RAD);
	}

	
};
*/