#include "teach_play/robot.h"


Robot::Robot(ros::NodeHandle *nh):
	as(*nh, "/move_linear_abs", boost::bind(&Robot::Execute, this, _1), false)
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

    if( !nh->getParam("/max_velocity", _max_velocity))
    	ROS_ERROR("Failed to get parameter from server.");

	if( !nh->getParam("/digital_input_number", _digital_input_number))
    	ROS_ERROR("Failed to get parameter from server.");

    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
   	robot_pose_pub = nh->advertise<geometry_msgs::Twist>("/robot_states", 10);

    control_mode_service = nh->advertiseService("/select_mode_service", &Robot::SelectModeCallback, this);
    remember_pts_service = nh->advertiseService("/remember_pts_service", &Robot::RememberPtsCallback, this);
    clear_pts_service = nh->advertiseService("/clear_pts_service", &Robot::ClearPtsCallback, this);
	laser_manager_service = nh->advertiseService("/laser_manager_service", &Robot::LaserManagerCallback, this);

	_straight_position = {0, 90, 90, 0, 90, 0};
	for(int i =0; i<JNT_NUM;i++)
	{
		_straight_points.push_back(_zero_points[i] + (_straight_position[i] / DEG_PER_REV) * _enc_resolution * _gear_ratios[i]);
	}
	_straight_queue.push_back(_straight_points);
	
	as.start();

    torque_mode_ready_flag = false;
	
	_sensor_flag = false;

	_play_points.clear();

    for(int i = 0; i<JNT_NUM; i++)
    {
    	_axis_deg.push_back(0);
		_robot_pose.push_back(0);
		_enc_cnts.push_back(0);
		_axis_torque_cmd.push_back(0);
		_vel_dir.push_back(0);
		_enc_cnts_tmp.push_back(0);
		_vel_dir_tmp.push_back(0);
	}

}

Robot::~Robot()
{
	_straight_points.clear();
	_straight_queue.clear();
	_axis_deg.clear();
	_robot_pose.clear();
	_enc_cnts.clear();
	_axis_torque_cmd.clear();
	_vel_dir.clear();
	_play_points.clear();
	delete ElmoMaster;
}


void Robot::JointStatesPublisher()
{
	bool res = ElmoMaster->ReadENC(_enc_cnts, _vel_dir);
	if (res != true)
	{	
		return;
	}
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
	bool select_mode_result = ElmoMaster->SelectModeProcess(req.data, torque_mode_ready_flag); //0(false for position mode) 1(true for torque mode)
	if (req.data == true)
	{
		res.message = "change to torque mode";
		_sensor_flag = false;
	}
	else
	{
		res.message = "change to position mode";
	}
	if (select_mode_result == true)
	{
		res.success = true;
	}
	else
	{
		res.success = false;
	}
    return true;
}


bool Robot::RememberPtsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	cout << _enc_cnts[0] << ", "
		 <<	_enc_cnts[1] << ", "
		 <<	_enc_cnts[2] << ", "
		 <<	_enc_cnts[3] << ", "
		 <<	_enc_cnts[4] << ", "
		 <<	_enc_cnts[5] << endl;

	_play_points.push_back(_enc_cnts);
	res.message = "Get point";
	res.success = true;
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
	robot_pose[0] = _T06(0,3);
	robot_pose[1] = _T06(1,3);
	robot_pose[2] = _T06(2,3);
	robot_pose[4] = RAD2DEG * atan2(-_T06(2, 0), sqrt(_T06(0, 0) * _T06(0, 0) + _T06(1, 0) * _T06(1, 0))); //pitch
    robot_pose[3] = RAD2DEG * atan2(_T06(2, 1) / cos(robot_pose[4]), _T06(2, 2) / cos(robot_pose[4])); //roll
    robot_pose[5] = RAD2DEG * atan2(_T06(1, 0) / cos(robot_pose[4]), _T06(0, 0) / cos(robot_pose[4])); //yaw
}

void Robot::IK(vector<double> &robot_pose, vector<double> &axis_deg, vector<double> &current_position)
{
	axis_deg.clear();
	
	double roll = robot_pose[3]/RAD2DEG; double pitch = robot_pose[4]/RAD2DEG; double yaw = robot_pose[5]/RAD2DEG;
	double wrist_x = robot_pose[0]- _d[5]*(cos(roll)*cos(yaw)*sin(pitch)+sin(roll)*sin(yaw));
	double wrist_y = robot_pose[1]- _d[5]*(-cos(yaw)*sin(roll)+cos(roll)*sin(pitch)*sin(yaw));
	double wrist_z = robot_pose[2]- _d[5]*cos(pitch)*cos(roll);
	double t1 = atan2(wrist_y, wrist_x);
	double t1_tmp = atan2(-wrist_y, -wrist_x);
	double axis1_position = ChooseNearst(t1,t1_tmp, current_position[0]/RAD2DEG); //axis 1
	
	double x_dot = wrist_x * cos(axis1_position) + wrist_y * sin(axis1_position);
	double y_dot = wrist_y * cos(axis1_position) - wrist_x * sin(axis1_position);
	double z_dot = (wrist_z - _d[0]);
	double t3 = asin((x_dot*x_dot+z_dot*z_dot-_a[2]*_a[2]-_d[3]*_d[3])/(2*_a[2]*_d[3]));
	double t3_tmp = PI-t3;
	double axis3_position = ChooseNearst(t3,t3_tmp, current_position[2]/RAD2DEG); //axis2
	
	double f1 = _a[2] + _d[3]*sin(axis3_position);
	double f2 = -_d[3]*cos(axis3_position);
	double axis2_position;
	if (f2 * f2 + f1 * f1 >= x_dot * x_dot)
    {	
    	double u = (-f2 * x_dot + f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);
    	double u_tmp = (-f2 * x_dot - f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);       
    	if (f2 == 0) 
    		axis2_position=0;
    	else
    	{
        	double t2 = atan2(u, (x_dot + f2 * u) / f1);
        	double t2_tmp = atan2(u_tmp, (x_dot + f2 * u_tmp) / f1);
        	axis2_position = ChooseNearst(t2,t2_tmp, current_position[1]/RAD2DEG);
        }
    }

    Matrix4d T06, inv_T03, T36;

    T06 << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), robot_pose[0],
       sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), robot_pose[1],
        -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), robot_pose[2],
       0,0,0,1;
	
	inv_T03 << (cos(axis1_position)*cos(axis2_position + axis2_position)), (cos(axis2_position + axis3_position)*sin(axis1_position)), (sin(axis2_position + axis3_position)), (-_a[2]*cos(axis3_position) -_d[0]*sin(axis2_position + axis3_position)),
          (-cos(axis1_position)*sin(axis2_position + axis3_position)), (-sin(axis1_position)*sin(axis2_position + axis3_position)), (cos(axis2_position + axis3_position)), (_a[2]*sin(axis3_position) - _d[0]*cos(axis2_position + axis3_position)),
          (sin(axis1_position)), (-cos(axis1_position)), 0, 0,
                0, 0, 0, 1;
	
	T36  = inv_T03*T06;

	double axis4_position, axis5_position, axis6_position;

	if (T36(1, 2) != -1 && T36(1,2) != 1)
    {
    	double t5 = atan2(sqrt(T36(1,0)*T36(1,0)+T36(1,1)*T36(1,1)), -T36(1,2));
    	double t5_tmp = atan2(-sqrt(T36(1,0)*T36(1,0)+T36(1,1)*T36(1,1)), -T36(1,2));
    	axis5_position = ChooseNearst(t5,t5_tmp, current_position[4]/RAD2DEG);

    	axis4_position = atan2(-T36(2,2)/sin(axis5_position), -T36(0,2)/sin(axis5_position));
    	axis6_position = atan2(T36(1,1)/sin(axis5_position), -T36(1,0)/sin(axis5_position));
    }
    else
    {
    	//singular point
    	axis4_position = 0.5 * atan2(T36(2, 0), T36(0, 0));
    	axis5_position = 0;
    	axis6_position = axis4_position;
	}

	axis_deg.push_back(axis1_position*RAD2DEG);
	axis_deg.push_back(axis2_position*RAD2DEG);
	axis_deg.push_back(axis3_position*RAD2DEG);
	axis_deg.push_back(axis4_position*RAD2DEG);
	axis_deg.push_back(axis5_position*RAD2DEG);
	axis_deg.push_back(axis6_position*RAD2DEG);

	
  	cout << axis_deg[0] << " , "
    << axis_deg[1] << " , "
    << axis_deg[2] << " , "
    << axis_deg[3] << " , "
    << axis_deg[4] << " , "
    << axis_deg[5] << " , " << endl;
}

double Robot::ChooseNearst(double a, double b, double c)
{
	double res;
	if (abs(a - c) <= abs(b - c))
        res = a;
    else
        res = b;
    return res;
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
		#if 1
		if (_axis_torque_cmd[i] >= 20000)
		{
			_axis_torque_cmd[i] = 0;
			cout << "torque exceed limit" << endl;
		}
		#endif
	}

	ElmoMaster->MoveTorque(_axis_torque_cmd);
}

void Robot::GravityComp(array<double, JNT_NUM> &g_torque, vector<double> &axis_deg)
{
	
	for(int i = 0; i<JNT_NUM; i++)
	{
		g_torque[i] = 0;
	}
	#if 1
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
  	R23(2,0) = _T23(2,0);R23(2,1) = _T23(2,1);R23(2,2) = _T23(2,2);

	Vector3d e02 = R01*R12*e;
	Vector3d e03 = R01*R12*R23*e;
	//first calculate axis 2 torque
	//gravity torque caused by axis3 on axis2

	Vector4d rAB2  = _T23*o;
  	Vector3d rAB(rAB2(0,0),rAB2(1,0), rAB2(2,0));
  	Vector3d rAB_0 =  R01*R12*rAB; //vector AB view as frame 0;
  	Vector3d F3(0,0,-5*9.81);
  	Vector3d T32 = rAB_0.cross(F3);
  	double T32_real = T32.dot(e02);

	//gravity torque caused by axis456 on axis2
	Vector4d rBC3 = _T34*_T45*o;
  	Vector3d rBC(rBC3(0,0), rBC3(1,0), rBC3(2,0));
  	Vector3d rBC_0 = R01*R12*R23*rBC;
  	Vector3d rAC_0 = rAB_0 + rBC_0;
  	Vector3d F4(0,0,-(7.5*9.81));
  	Vector3d T42 = rAC_0.cross(F4);
  	double T42_real = T42.dot(e02);

	//gravity torque caused by axis456 on axis3
	Vector3d T43 = rBC_0.cross(F4);
  	double T43_real = T43.dot(e03);

	g_torque[1] = -1000 * (((T32_real + T42_real) / _gear_ratios[1])/_motor_torque_const[1])/HD_eff;
	g_torque[2] = -1000 * ((T43_real/ _gear_ratios[2]) / _motor_torque_const[2])/HD_eff; 
	//cout << "axis2 motor g torque" << -1000 * ((T32_real + T42_real) / _gear_ratios[1])/_motor_torque_const[1] << endl;
	//cout << "axis3 motor g torque" << (T43_real/ _gear_ratios[2]) / _motor_torque_const[2]; 

	#endif
}

void Robot::AuxComp(array<double, JNT_NUM> &aux_torque, vector<int> &vel_dir)
{
	double vel_factor;
	DIState left_button_state = ElmoMaster->GetDISignal(_digital_input_number[0]);
	DIState right_button_state = ElmoMaster->GetDISignal(_digital_input_number[1]);

	switch(left_button_state)
	{
		case BUTTON_PRESSED:
			vel_factor = 2;
			break;
		case BUTTON_NON_PRESSED:
			vel_factor = 1;
			break;
	}


	for(int i = 0; i<JNT_NUM; i++)
	{
		if (vel_dir[i] == VelDir::POSITIVE)
		{
			if(i > 2)
			{
				vel_factor = 1;
			}
			aux_torque[i] = _motor_friction_current[i] * vel_factor;
		}
		else if (vel_dir[i] == VelDir::NEGATIVE)
		{
			if(i > 2)
			{
				vel_factor = 1;
			}
			aux_torque[i] = -_motor_friction_current[i] * vel_factor; //only plus axis123
		}
		else
		{
			aux_torque[i] = 0;
		}
	}

	switch(right_button_state)
	{
		case BUTTON_PRESSED:
			aux_torque[4] = 0;
			aux_torque[2] = 0;
			break;
		case BUTTON_NON_PRESSED:
			vel_factor = 1;
			break;
	}
}



bool Robot::ClearPtsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	_play_points.clear();
	res.success = true;
	res.message = "clear points";
    return true;
}



bool Robot::LaserManagerCallback(teach_play::LaserManager::Request &req, teach_play::LaserManager::Response &res)
{
	
	if (_sensor_flag == true)
	{
		string current_zone = req.zone;
		ElmoMaster->SetSpeedOverride(req.vel_factor);
		res.message = "SpeedOverride is set";
	}
	res.success = true;
	return true;
}



void Robot::Execute(const teach_play::MoveLinearAbsGoalConstPtr& goal)
{
	ROS_INFO("Recieve action successful!");
  	
  	if (ElmoMaster->GetDriverMode() != DriverMode::CSP)
	{
		bool select_mode_result = ElmoMaster->SelectModeProcess(false, torque_mode_ready_flag);

		if (select_mode_result != true)
		{
			_result.success = false;
			_result.message = "change op mode failed";
			as.setPreempted(_result);
		}
	}
	
		
	
	double vel = _max_velocity*0.01*goal->vel;
	double vel_tmp = _max_velocity*0.1;
	MotionType type;

	switch(goal->type)
	{ 
    	case 0:
    		type = GO_POINTS;
    		_sensor_flag = true;
			ElmoMaster->GroupLinearMotionMove(_play_points, vel);
        	break;
        case 1:
        	type = GO_STRAIGHT;
        	ElmoMaster->GroupLinearMotionMove(_straight_queue, vel_tmp);
        	break;
        case 2:
        	type = JOG;
        	GetJogGoal();
        	ElmoMaster->GroupLinearMotionMove(_jog_goal_queue, vel_tmp);
        	break;
	}
		
	ros::Rate rate(10);
	GroupState state;
	while(1)
	{	
		state = ElmoMaster->CheckGroupStatus();
		if (state == STOP)
		{
			break;
		}

		//_feedback.point_num = _play_points.size();
		//_feedback.zone = _current_zone;
		//as.publishFeedback(_feedback);
		ros::spinOnce();
		rate.sleep();
	}
	_result.success = true;
	_result.message = "motion succeed";
	as.setSucceeded(_result);
	_sensor_flag = false;
	
}

void Robot::GetJogGoal()
{
	
}