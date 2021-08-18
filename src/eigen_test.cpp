#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include "teach_play/def.h"

using namespace std;
using namespace Eigen;
 
#define DEG2RAD 0.017453

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eigen_test");
  VelDir veldir = VelDir::NEGATIVE;
  Matrix4d T01 = GetTFMatrix(0, 1);
  Matrix4d T12 = GetTFMatrix(90, 2);
  Matrix4d T23 = GetTFMatrix(0, 3);
  Matrix4d T34 = GetTFMatrix(0, 4);
  Matrix4d T45 = GetTFMatrix(90, 5);
  Matrix4d T56 = GetTFMatrix(0, 6);
  Matrix4d T06 = T01*T12*T23*T34*T45*T56;
  cout << T06 << endl;
}