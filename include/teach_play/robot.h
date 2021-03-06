//include ros library
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <teach_play/MotionPlanning.h>
#include <teach_play/LaserManager.h>
#include <teach_play/Decode.h>
#include <actionlib/server/simple_action_server.h>

#include <teach_play/MoveLinearAbsAction.h>

#include "teach_play/hardware_transmission_interface.h"

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <string>
#include <cmath>
#include <deque>
#include <unistd.h>

#define JNT_NUM 6
#define PI 3.14157
#define DEG_PER_REV 360
#define DEG2RAD 0.017453
#define RAD2DEG 57.2967
#define g 9.81
#define HD_eff 0.7

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
	ros::ServiceServer remember_pts_service;
	//ros::ServiceServer play_pts_service;
	ros::ServiceServer go_straight_service;
	ros::ServiceServer clear_pts_service;
	ros::ServiceServer laser_manager_service;
	ros::ServiceServer jog_goal_service;
	actionlib::SimpleActionServer<teach_play::MoveLinearAbsAction> as;

	Robot(ros::NodeHandle *nh);
	~Robot();
	void JointStatesPublisher();
	void RobotPosePublisher();
	bool SelectModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	bool RememberPtsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool ClearPtsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool JogGoalCallback(teach_play::Decode::Request &req, teach_play::Decode::Response &res);

	bool LaserManagerCallback(teach_play::LaserManager::Request &req, teach_play::LaserManager::Response &res);
	bool PlayPtsCallback(teach_play::MotionPlanning::Request &req, teach_play::MotionPlanning::Response &res);
	void UpdateTorque();
	void Execute(const teach_play::MoveLinearAbsGoalConstPtr& goal);

	teach_play::MoveLinearAbsFeedback _feedback;
	teach_play::MoveLinearAbsResult _result;

	bool torque_mode_ready_flag;




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
	vector<int> _digital_input_number;
	int _enc_resolution; //encoder counts per rev
	int _max_velocity; //maximum velocity of joint vel, cnts/sec


	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56 , _T06;
	vector<double> _axis_deg;
	vector<double> _robot_pose;
	vector<double> _axis_torque_cmd;

	vector<double> _goal_position;
	vector<double> _goal_pose;

	vector<double> _enc_cnts;
	vector<int> _vel_dir;
	vector<double> _enc_cnts_tmp;
	vector<int> _vel_dir_tmp;
	
	deque<vector<double>> _play_points;
	bool _sensor_flag;
	array<double, JNT_NUM> _straight_position;
	vector<double> _straight_points;
	deque<vector<double>> _straight_queue;
	deque<vector<double>> _jog_goal_queue;

	void FK(vector<double> &robot_pose, vector<double> &axis_deg);
	void IK(vector<double> &robot_pose, vector<double> &axis_deg, vector<double> &current_position);
	double ChooseNearst(double a, double b, double c);
	Matrix4d GetTFMatrix(double axis_deg, int id);
	void GravityComp(array<double, JNT_NUM> &g_torque, vector<double> &axis_deg);
	void AuxComp(array<double, JNT_NUM> &aux_torque, vector<int> &vel_dir);


};
