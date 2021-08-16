#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include "teach_play/def.h"

using namespace std;
using namespace Eigen;
 

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eigen_test");
  VelDir veldir = VelDir::NEGATIVE;
  //int a = veldir;
  cout << veldir << endl;


  /*	ros::NodeHandle nh;
  	Matrix4d m;
  	m << 1,2,3,4,
  		5,6,7,8,
  		9,10,11,12,
  		13,14,15,16;
  	Vector4d o(0,0,0,1);
	Vector3d e(0,0,1);

	MatrixXd r23_0 = m*o;
	cout << r23_0 << endl;*/

}