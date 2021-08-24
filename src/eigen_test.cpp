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
#define JNT_NUM 6

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

void GravityComp(array<double, JNT_NUM> &g_torque, array<double, JNT_NUM> &axis_deg)
{

  Matrix4d _T01 = GetTFMatrix(axis_deg[0], 1);
  Matrix4d _T12 = GetTFMatrix(axis_deg[1], 2);
  Matrix4d _T23 = GetTFMatrix(axis_deg[2], 3);
  Matrix4d _T34 = GetTFMatrix(axis_deg[3], 4);
  Matrix4d _T45 = GetTFMatrix(axis_deg[4], 5);
  Matrix4d _T56 = GetTFMatrix(axis_deg[5], 6);
  
  for(int i = 0; i<JNT_NUM; i++)
  {
    g_torque[i] = 0;
  }
  
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

  //cout << e02 << endl;
  //cout << R23 << endl;
  cout << "axis2 torque = " << T32_real + T42_real << endl;
  cout << "axis3 torque = " << T43_real << endl;
  #if 0
  //gravity torque caused by axis456 on axis2
  MatrixXd rAC_0 = rAB_0 + _T01*_T12*_T23*_T34*_T45*o; //AC = AB+BC
  Vector3d rAC(rAC_0(0,0),rAC_0(1,0), rAC_0(2,0));
  Vector3d F4(0,0,-(7.5*9.81));
  Vector3d T42 = rAC.cross(F4);
  double T42_real = T42.dot(e02);

  //gravity torque caused by axis456 on axis3
  MatrixXd rBC_0 = _T01*_T12*_T23*_T34*_T45*o;
  Vector3d rBC(rBC_0(0,0),rBC_0(1,0), rBC_0(2,0));
  Vector3d T43 = rBC.cross(F4);
  double T43_real = T43.dot(e03);
  cout << rAB_0 << endl;
  //cout << "========" << endl;
  //cout << T43 << endl;
  //cout << "axis2 torque = " << T32_real + T42_real << endl;
  //cout << "axis3 torque = " << T43_real << endl;
  //g_torque[1] = -1000 * ((T32_real + T42_real) / _gear_ratios[1])/_motor_torque_const[1];
  //g_torque[2] = -1000 * (T43_real/ _gear_ratios[2]) / _motor_torque_const[2]; 
  //cout << "axis2 motor g torque" << -1000 * ((T32_real + T42_real) / _gear_ratios[1])/_motor_torque_const[1] << endl;
  //cout << "axis3 motor g torque" << (T43_real/ _gear_ratios[2]) / _motor_torque_const[2]; 

  #endif
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eigen_test");
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