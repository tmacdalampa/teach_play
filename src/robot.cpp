#include "teach_play/robot.h"


Robot::Robot(ros::NodeHandle *nh)
{	
	ElmoMaster = new HwTmIntf;

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

    if( !nh->getParam("/M", _M))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/motor_torque_const", _motor_torque_const))
    	ROS_ERROR("Failed to get parameter from server.");

    if( !nh->getParam("/motor_friction_current", _motor_friction_current))
    	ROS_ERROR("Failed to get parameter from server.");

    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
   	robot_pose_pub = nh->advertise<geometry_msgs::Twist>("/robot_states", 10);

    control_mode_service = nh->advertiseService("/select_mode_service", &Robot::SelectModeCallback, this);

    torque_mode_flag = false;
    for(int i = 0; i<JNT_NUM; i++)
    {
    	_axis_deg.push_back(0);
		_robot_pose.push_back(0);
		_enc_cnts.push_back(0);
		_axis_torque_cmd.push_back(0);
		_vel_dir.push_back(0);
	}

}

Robot::~Robot()
{
	_axis_deg.clear();
	_robot_pose.clear();
	_enc_cnts.clear();
	_axis_torque_cmd.clear();
	_vel_dir.clear();
	delete ElmoMaster;
}


void Robot::JointStatesPublisher()
{
	ElmoMaster->ReadENC(_enc_cnts, _vel_dir);
	
	for (int i = 0; i < JNT_NUM; i++)
    {
    	_axis_deg[i] = (((_enc_cnts[i]- _zero_points[i])/_gear_ratios[i]) *DEG_PER_REV) /_enc_resolution;
	}

	sensor_msgs::JointState joint_states;
    std_msgs::Header t;

	t.stamp = ros::Time::now();
  	joint_states.header = t; 
  	joint_states.position = _axis_deg;	
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
	ElmoMaster->ChangeOpMode(req.data); //0(false for position mode) 1(true for torque mode)
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
	_T01 = GetTFMatrix(axis_deg[0], 1);
	_T12 = GetTFMatrix(axis_deg[1], 2); 
	_T23 = GetTFMatrix(axis_deg[2], 3); 
	_T34 = GetTFMatrix(axis_deg[3], 4);
	_T45 = GetTFMatrix(axis_deg[4], 5);
	_T56 = GetTFMatrix(axis_deg[5], 6);
	_T06 = _T01*_T12*_T23*_T34*_T45*_T56;
	
	robot_pose = {_T06(0,3),_T06(1,3),_T06(2,3),0,0,0};
}

void Robot::UpdateTorque()
{
	array<double, JNT_NUM> g_torque;
	array<double, JNT_NUM> aux_torque;
	GravityComp(g_torque, _axis_deg);
	AuxComp(aux_torque, _vel_dir);

	for(int i = 0; i<JNT_NUM; i++)
	{
		_axis_torque_cmd[i] = g_torque[i] + aux_torque[i];
	}

	ElmoMaster->MoveTorque(_axis_torque_cmd);
}

void Robot::GravityComp(array<double, JNT_NUM> &g_torque, vector<double> &axis_deg)
{
	
	for(int i = 0; i<JNT_NUM; i++)
	{
		g_torque[i] = 0;
	}
	#if 0
	//assume O as oroginal point of arm, A as origin point of frame 1 and 2, B as origin point of frame 3, C as interaction of axis456, D as TCP point
	Vector4d o(0,0,0,1);
	Vector3d e(0,0,1);
	Matrix3d R01, R12, R23;
	R01(0,0) = _T01(0,0);R01(0,1) = _T01(0,1);R01(0,2) = _T01(0,2);
	R01(1,0) = _T01(1,0);R01(1,1) = _T01(1,1);R01(1,2) = _T01(1,2);
	R01(2,0) = _T01(2,0);R01(2,1) = _T01(2,1);R01(2,2) = _T01(2,2);

	R12(0,0) = _T12(0,0);R12(0,1) = _T12(0,1);R12(0,2) = _T12(0,2);
	R12(1,0) = _T12(1,0);R12(1,1) = _T12(1,1);R12(1,2) = _T12(1,2);
	R12(2,0) = _T12(2,0);R12(2,1) = _T12(2,1);R12(2,2) = _T12(2,2);

	R23(0,0) = _T23(0,0);R23(0,1) = _T23(0,1);R23(0,2) = _T23(0,2);
	R23(1,0) = _T23(1,0);R23(1,1) = _T23(1,1);R23(1,2) = _T23(1,2);
	R23(2,0) = _T23(2,0);R23(2,1) = _T23(2,1);R23(2,2) = _T12(2,2);

	Vector3d e02 = R01*R12*e;
	Vector3d e03 = R01*R12*R23*e;
	//first calculate axis 2 torque
	//gravity torque caused by axis3 on axis2

	MatrixXd rAB_0 = _T01*_T12*_T23*o; //vector AB view as frame 0;
	Vector3d rAB(rAB_0(0,0),rAB_0(1,0), rAB_0(2,0));

	Vector3d F3(0,0,-_M[2]*g);

	Vector3d T32 = rAB.cross(F3);
	double T32_real = T32.dot(e02);

	//gravity torque caused by axis456 on axis2
	MatrixXd rAC_0 = rAB_0 + _T01*_T12*_T23*_T34*_T45*o; //AC = AB+BC
	Vector3d rAC(rAC_0(0,0),rAC_0(1,0), rAC_0(2,0));
	Vector3d F4(0,0,-(_M[3]+_M[4]+_M[5])*g);
	Vector3d T42 = rAC.cross(F4);
	double T42_real = T42.dot(e02);

	//gravity torque caused by axis456 on axis3
	MatrixXd rBC_0 = _T01*_T12*_T23*_T34*_T45*o;
	Vector3d rBC(rBC_0(0,0),rBC_0(1,0), rBC_0(2,0));
	Vector3d T43 = rBC.cross(F4);
	double T43_real = T43.dot(e03);

	g_torque[1] = 1000 * (T32_real + T42_real)/_motor_torque_const[1];
	g_torque[2] = 1000 * T43_real/_motor_torque_const[2];
	#endif
}

void Robot::AuxComp(array<double, JNT_NUM> &aux_torque, vector<int> &vel_dir)
{
	for(int i = 0; i<JNT_NUM; i++)
	{
		if (vel_dir[i] == VelDir::POSITIVE)
		{
			aux_torque[i] = _motor_friction_current[i];
		}
		else if (vel_dir[i] == VelDir::NEGATIVE)
		{
			aux_torque[i] = -_motor_friction_current[i];
		}
		else
		{
			aux_torque[i] = 0;
		}
	}
}