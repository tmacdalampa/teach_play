#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eigen_test");
  	ros::NodeHandle nh;
	Matrix4f m;
	m << 1, 2, 3, 4,
     	 5, 6, 7, 8,
     	 9,10,11,12,
     	 13,14,15,16;
	cout << m << endl;

	Matrix3f n;
	n(0,0) = m(0,0); n(0,1) = m(0,1); n(0,2) = m(0,2);
	n(1,0) = m(1,0); n(1,1) = m(1,1); n(1,2) = m(1,2);
	n(2,0) = m(2,0); n(2,1) = m(2,1); n(2,2) = m(2,2);
	cout << n << endl;

}