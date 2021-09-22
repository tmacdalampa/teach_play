#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <array>
#include <vector>
#include <deque>
#include "teach_play/def.h"

using namespace std;
using namespace Eigen;
 
#define DEG2RAD 0.017453
#define RAD2DEG 57.2967
#define JNT_NUM 6
#define PI 3.14159

Matrix4d GetTFMatrix(double axis_deg, int id)
{
  Matrix4d out_T;
  double _A[6] = {0,0,0.425,0,0,0};
  double _D[6] = {0.426, 0, 0, 0.425, 0, 0.0755};
  double _alpha[6] = {0, 90, 0, 90, 90, -90};
  double sa, ca, cs, ss;

  id = id - 1;

  //A = _a[id];
  //D = _d[id];
  
  sa = sin( _alpha[id] * DEG2RAD);
  ca = cos( _alpha[id] * DEG2RAD);
  
  ss = sin( axis_deg * DEG2RAD);
  cs = cos( axis_deg * DEG2RAD);

  out_T(0,0) =    cs; out_T(0,1) =   -ss; out_T(0,2) =   0;    out_T(0,3) =   _A[id];
  out_T(1,0) = ss * ca; out_T(1,1) = cs * ca;   out_T(1,2) = -sa;    out_T(1,3) = -sa * _D[id];
  out_T(2,0) = ss * sa; out_T(2,1) = cs * sa;   out_T(2,2) =  ca;    out_T(2,3) =  ca * _D[id];  
  out_T(3,0) =     0; out_T(3,1) =     0;   out_T(3,2) =   0;    out_T(3,3) =   1;
      
  /*
      [      cos(θ)          -sin(θ)     0          A ]
    T = [ sin(θ) * cos(α)    cos(θ) * cos(α)  -sin(α)   -sin(α) * D ]
      [ sin(θ) * sin(α)    cos(θ) * sin(α)   cos(α)    cos(α) * D ]
      [        0            0      0          1 ]
  */
  return out_T;
}


double ChooseNearst(double a, double b, double c)
{
	double res;
	if (abs(a - c) <= abs(b - c))
        res = a;
    else
        res = b;
    return res;
}

void IK(vector<double> &robot_pose, vector<double> &axis_deg, vector<double> &current_position)
{
	axis_deg.clear();
	double d6 = 0.0775; double d4 = 0.425; double a3 = 0.425; double d1 = 0.426;
	double roll = robot_pose[3]/RAD2DEG; double pitch = robot_pose[4]/RAD2DEG; double yaw = robot_pose[5]/RAD2DEG;
	double wrist_x = robot_pose[0]- d6*(cos(roll)*cos(yaw)*sin(pitch)+sin(roll)*sin(yaw));
	double wrist_y = robot_pose[1]- d6*(-cos(yaw)*sin(roll)+cos(roll)*sin(pitch)*sin(yaw));
	double wrist_z = robot_pose[2]- d6*cos(pitch)*cos(roll);
	double t1 = atan2(wrist_y, wrist_x);
	double t1_tmp = atan2(-wrist_y, -wrist_x);
	double axis1_position = ChooseNearst(t1,t1_tmp, current_position[0]/RAD2DEG); //axis 1
	
	double x_dot = wrist_x * cos(axis1_position) + wrist_y * sin(axis1_position);
	double y_dot = wrist_y * cos(axis1_position) - wrist_x * sin(axis1_position);
	double z_dot = (wrist_z - d1);
	double t3 = asin((x_dot*x_dot+z_dot*z_dot-a3*a3-d4*d4)/(2*a3*d4));
	double t3_tmp = PI-t3;
	double axis3_position = ChooseNearst(t3,t3_tmp, current_position[2]/RAD2DEG); //axis2
	
	double f1 = a3 + d4*sin(axis3_position);
	double f2 = -d4*cos(axis3_position);
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

    T06 << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), robot_pose[0];
       sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), robot_pose[1];
        -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), robot_pose[2];
       0,0,0,1;
	
	inv_T03 << (cos(axis1_position)*cos(axis2_position + axis2_position)), (cos(axis2_position + axis3_position)*sin(axis1_position)), (sin(axis2_position + axis3_position)), (-a3*cos(axis3_position) -d1*sin(axis2_position + axis3_position));
          (-cos(axis1_position)*sin(axis2_position + axis3_position)), (-sin(axis1_position)*sin(axis2_position + axis3_position)), (cos(axis2_position + axis3_position)), (a3*sin(axis3_position) - d1*cos(axis2_position + axis3_position));
          (sin(axis1_position)), (-cos(axis1_position)), 0, 0;
                0, 0, 0, 1;
	
	T36  = inv_T03*T06;

	double axis4_position, axis5_position, axis6_position;

	if (T36(1, 2) != -1 && T36(1,2) != 1)
    {
    	double t5 = atan2(sqrt(T36(1,0)*T36(1,0)+T36(1,1)*T36(1,1)), -T36(1,2));
    	double t5_tmp = atan2(-sqrt(T36(1,0)*T36(1,0)+T36(1,1)*T36(1,1)), -T36(1,2));
    	axis5_position = ChooseNearst(t5,t5_tmp, current_position[4]/RAD2DEG);

    	axis4_position = atan2(-T36(1,1)/sin(axis5_position), -T36(0,2)/sin(axis5_position));
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

	for(int i = 0; i < 6; i++)
	{
  		cout << axis_deg[0] << " , "
       << axis_deg[1] << " , "
       << axis_deg[2] << " , "
       << axis_deg[3] << " , "
       << axis_deg[4] << " , "
       << axis_deg[5] << " , " << endl;
	}
}

void Robot::FK(vector<double> &robot_pose, vector<double> &axis_deg)
{
	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56 , _T06;
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

int main(int argc, char** argv)
{
	array<double, 6> robot_pose;
	array<double, 6> axis_deg = {0, 90, 90, 0, 90, 0};
	ros::init(argc, argv, "eigen_test");
	/*
	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56 , _T06;
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
    cout << "x = " << robot_pose[0]
    	<<  ", y = " << robot_pose[1]
    	<<  ", z = " << robot_pose[2]
    	<<  ", roll = " << robot_pose[3]
    	<<  ", pitch = " << robot_pose[4]
    	<<  ", yaw = " << robot_pose[5] << endl;
	*/
    #if 0
  array<double, JNT_NUM> g_torque = {0,0,0,0,0,0};
  vector<double> axis_deg1 = {0, 0, 0, 0, 0, 0};
  vector<double> axis_deg2 = {1000, 1000, 1000, 1000, 1000, 1000};
  vector<double> axis_deg3 = {2500, 2500, 2500, 2500, 2500, 2500};
  vector<double> axis_deg4 = {3000, 3000, 3000, 3000, 3000, 3000};
  vector<double> axis_deg5 = {5000, 5000, 5000, 5000, 5000, 5000};
  int max_vel = 1000;

  deque<vector<double>> play_points;
  double dTable[170];

  play_points.push_back(axis_deg1);
  play_points.push_back(axis_deg2);
  play_points.push_back(axis_deg3);
  play_points.push_back(axis_deg4);
  play_points.push_back(axis_deg5);
  int point_num = play_points.size();


  for(int i = 0; i < point_num +1; i++)
    {
        if (i == 0)
        {
            dTable[13*i] = 0; //t0=0
      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0
                vector<double> six_axis_pt = play_points.back();
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
            }
        }
        else
        {
            vector<double> path_each_joint;
            vector<double> six_axis_pt = play_points[i-1];      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0    
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
                path_each_joint.push_back(abs(dTable[13*i + 2*(j+1)-1] - dTable[13*(i-1) + 2*(j+1)-1]));
            }
            double max_path = *max_element(path_each_joint.begin(), path_each_joint.end());

            dTable[13*i] = max_path/max_vel;
            //cout << "this path time interval = " << max_path/max_vel << endl;
        }
    }
	#endif
    #if 0
for(int i = 0; i<point_num+1; i++)
{
  cout << dTable[13*i] << " , "
       << dTable[13*i+1] << " , "
       << dTable[13*i+2] << " , "
       << dTable[13*i+3] << " , "
       << dTable[13*i+4] << " , "
       << dTable[13*i+5] << " , "
       << dTable[13*i+6] << " , "
       << dTable[13*i+7] << " , "
       << dTable[13*i+8] << " , "
       << dTable[13*i+9] << " , "
       << dTable[13*i+10] << " , "
       << dTable[13*i+11] << " , "
       << dTable[13*i+12] << " , " << endl;
}

    
    MatrixXd Axis1_B_Matrix(point_num-1, 1);
    MatrixXd Axis2_B_Matrix(point_num-1, 1);
    MatrixXd Axis3_B_Matrix(point_num-1, 1);
    MatrixXd Axis4_B_Matrix(point_num-1, 1);
    MatrixXd Axis5_B_Matrix(point_num-1, 1);
    MatrixXd Axis6_B_Matrix(point_num-1, 1);
    MatrixXd Time_Matrix(point_num-1, point_num-1);

    for(int i = 0;i <= point_num - 2;i++)
    {
        for(int j = 0;j <= point_num - 2;j++)
        {
            Time_Matrix(i,j) = 0;
        }
        Axis1_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+1] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+1] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+1]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]);
        
        Axis2_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+3] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+3] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+3]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis3_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+5] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+5] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+5]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis4_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+7] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+7] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+7]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis5_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+9] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+9] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+9]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis6_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+11] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+11] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+11]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 

        cout << Axis1_B_Matrix(i, 0) << " , " << Axis6_B_Matrix(i, 0) << endl;
    }

    for (int i  = 0; i <= point_num-2; i++)
    {
        Time_Matrix(i, i) = 2 * (dTable[13*(i + 2)] + dTable[13*(i + 1)]);

        if (i > 0 && i < point_num - 2)
        {
            Time_Matrix(i, i - 1) = dTable[13*(i + 2)];
            Time_Matrix(i, i + 1) = dTable[13*(i + 1)];
        }
        else if (i == 0)
        {
            Time_Matrix(i, i + 1) = dTable[13*(i + 1)];
        }
        else if (i == point_num-2)
        {
            Time_Matrix(i, i - 1) = dTable[13*(i + 2)];
        }
    }

    MatrixXd Axis1_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis2_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis3_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis4_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis5_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis6_Vel_Matrix(point_num-1, 1);


    Axis1_Vel_Matrix = Time_Matrix.inverse() * Axis1_B_Matrix;
    Axis2_Vel_Matrix = Time_Matrix.inverse() * Axis2_B_Matrix;
    Axis3_Vel_Matrix = Time_Matrix.inverse() * Axis3_B_Matrix;
    Axis4_Vel_Matrix = Time_Matrix.inverse() * Axis4_B_Matrix;
    Axis5_Vel_Matrix = Time_Matrix.inverse() * Axis5_B_Matrix;
    Axis6_Vel_Matrix = Time_Matrix.inverse() * Axis6_B_Matrix;

    for(int i = 0; i<point_num-1; i++)
    {
        dTable[13*(i+1) + 2] = Axis1_Vel_Matrix(i,0);
        dTable[13*(i+1) + 4] = Axis2_Vel_Matrix(i,0);
        dTable[13*(i+1) + 6] = Axis3_Vel_Matrix(i,0);
        dTable[13*(i+1) + 8] = Axis4_Vel_Matrix(i,0);
        dTable[13*(i+1) + 10] = Axis5_Vel_Matrix(i,0);
        dTable[13*(i+1) + 12] = Axis6_Vel_Matrix(i,0);
    }
#endif
    #if 0
for(int i = 0; i<point_num+1; i++)
{
  cout << dTable[13*i] << " , "
       << dTable[13*i+1] << " , "
       << dTable[13*i+2] << " , "
       << dTable[13*i+3] << " , "
       << dTable[13*i+4] << " , "
       << dTable[13*i+5] << " , "
       << dTable[13*i+6] << " , "
       << dTable[13*i+7] << " , "
       << dTable[13*i+8] << " , "
       << dTable[13*i+9] << " , "
       << dTable[13*i+10] << " , "
       << dTable[13*i+11] << " , "
       << dTable[13*i+12] << " , " << endl;
}
#endif

  

  
  
}