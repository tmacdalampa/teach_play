//include ros library
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include "teach_play/hardware_transmission_interface.h"

#include <array>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <string>
#include <cmath>

#define JNT_NUM 6
#define PI 3.14157
#define DEG_PER_REV 360
#define DEG2RAD 0.0017453
#define g 9.81

using namespace std;
using namespace Eigen;

class HwTmIntf;
class Robot
{
public:
	HwTmIntf* ElmoMaster;

	ros::Publisher joint_state_pub;
	ros::Publisher robot_pose_pub;
	ros::ServiceServer control_mode_service;

	Robot(ros::NodeHandle *nh);
	~Robot();
	void JointStatesPublisher();
	void RobotPosePublisher();
	bool SelectModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	void UpdateTorque();

	bool torque_mode_flag;



private:
	//parameters from yaml file
	vector<int> _gear_ratios;
	vector<int> _zero_points; //unit cnts
	vector<double> _a; //unit meter
	vector<double> _alpha; //unit deg
	vector<double> _d; //unit meter
	
	vector<double> _M; //unit kg
	vector<double> _motor_torque_const; //unit Nm/Arms
	vector<double> _motor_friction_current; //unit mAmp
	int _enc_resolution;


	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56 , _T06;
	vector<double> _axis_deg;
	vector<double> _robot_pose;
	vector<double> _axis_torque_cmd;


	vector<double> _enc_cnts;
	vector<int> _vel_dir;

	void FK(vector<double> &robot_pose, vector<double> &axis_deg);
	Matrix4d GetTFMatrix(double axis_deg, int id);
	void GravityComp(vector<double> &g_torque, vector<double> &axis_deg);
	void AuxComp(vector<double> &aux_torque, vector<int> &vel_dir);


};
