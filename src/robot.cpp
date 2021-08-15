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

    if( !nh->getParam("/a", _a))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/alpha", _alpha))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/d", _d))
    	ROS_ERROR("Failed to get parameter from server.");

    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
   	robot_pose_pub = nh->advertise<geometry_msgs::Twist>("/robot_states", 10);

    control_mode_service = nh->advertiseService("/select_mode_service", &Robot::SelectModeCallback, this);

    torque_mode_flag = false;
    for(int i = 0; i<JNT_NUM; i++)
    {
    	_axis_deg[i] = 0;
		_robot_pose[i] = 0;
		_enc_cnts[i] = 0;
		_axis_torque_cmd[i] = 0;
	}
}

Robot::~Robot()
{

}


void Robot::JointStatesPublisher()
{
	_enc_cnts = ElmoMaster.ReadENC();
	
	for (int i = 0; i < JNT_NUM; i++)
    {
    	_axis_deg[i] = ((_enc_cnts[i]- _zero_points[i])/_gear_ratios[i])*( DEG_PER_REV /_enc_resolution);
	}

	sensor_msgs::JointState joint_states;
    std_msgs::Header t;

	t.stamp = ros::Time::now();
  	joint_states.header = t; 
  	joint_states.position = _enc_cnts;	
	joint_state_pub.publish(joint_states);
}

void Robot::RobotPosePublisher()
{
	FK(_robot_pose, _axis_deg);
	geometry_msgs::Twist robot_states;
	robot_states.linear.x = _robot_pose[0];
	robot_states.linear.y = _robot_pose[1];
	robot_states.linear.z = _robot_pose[2];
	robot_states.angular.x = _robot_pose[3];
	robot_states.angular.y = _robot_pose[4];
	robot_states.angular.z = _robot_pose[5];
	robot_pose_pub.publish(robot_states);
}


bool Robot::SelectModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	ElmoMaster.ChangeOpMode(req.data); //0(false for position mode) 1(true for torque mode)
	if (req.data == true)
	{
		torque_mode_flag = true;
		res.message = "torque mode done";
	}
	else
	{
		res.message = "position mode done";
	}
	res.success = true;

	cout << res.message << endl;
    return true;
}


Matrix4d Robot::GetTFMatrix(double axis_deg, int id)
{
	Matrix4d out_T;
	double A, D, sa, ca, cs, ss;

	id = id - 1;

	A = _a[id];
	D = _d[id];
	
	sa = sin( _alpha[id] * DEG2RAD);
	ca = cos( _alpha[id] * DEG2RAD);
	
	ss = sin( axis_deg * DEG2RAD);
	cs = cos( axis_deg * DEG2RAD);

	out_T(0,0) =	  cs;	out_T(0,1) =	 -ss;	out_T(0,2) =   0;    out_T(0,3) =		A;
	out_T(1,0) = ss * ca;	out_T(1,1) = cs * ca;   out_T(1,2) = -sa;    out_T(1,3) = -sa * D;
	out_T(2,0) = ss * sa;	out_T(2,1) = cs * sa;   out_T(2,2) =  ca;    out_T(2,3) =  ca * D;  
	out_T(3,0) =	   0;	out_T(3,1) =	   0;   out_T(3,2) =   0;    out_T(3,3) =		1;
	    
	/*
		  [			 cos(θ)		  		 -sin(θ)		 0				  A	]
	  T = [	sin(θ) * cos(α)		 cos(θ) * cos(α)	-sin(α)		-sin(α) * D	]
		  [	sin(θ) * sin(α)		 cos(θ) * sin(α)	 cos(α)		 cos(α) * D	]
		  [				 0					  0			 0				  1	]
	*/
	return out_T;
}

void Robot::FK(vector<double> &robot_pose, vector<double> &axis_deg)
{
	Matrix4d T01 = GetTFMatrix(axis_deg[0], 1);
	Matrix4d T12 = GetTFMatrix(axis_deg[1], 2); 
	Matrix4d T23 = GetTFMatrix(axis_deg[2], 3); 
	Matrix4d T34 = GetTFMatrix(axis_deg[3], 4);
	Matrix4d T45 = GetTFMatrix(axis_deg[4], 5);
	Matrix4d T56 = GetTFMatrix(axis_deg[5], 6);
	Matrix4d T06 = T01*T12*T23*T34*T45*T56;
	
	robot_pose = {T06(0,3),T06(1,3),T06(2,3),0,0,0};
}

void Robot::UpdateTorque()
{
	vector<double> g_torque;
	GravityComp(g_torque, _axis_deg);

	for(int i = 0; i<JNT_NUM; i++)
	{
		_axis_torque_cmd[i] = g_torque[i] + _aux_torque[i];
	}

	ElmoMaster.MoveTorque(_axis_torque_cmd);
}

void Robot::GravityComp(vector<double>&g_torque, vector<double> &axis_deg)
{
	
	for(int i = 0; i<JNT_NUM; i++)
	{
		g_torque[i] = 0;
	}
	

}
