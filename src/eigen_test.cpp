#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <array>

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
  array<double, JNT_NUM> axis_deg = {0,90,0,0,90,0};
  GravityComp(g_torque, axis_deg);
  //VelDir veldir = VelDir::NEGATIVE;
  
  
}