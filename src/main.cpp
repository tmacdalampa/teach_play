//main.cpp
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include "teach_play/robot.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cobot_controller");
  	ros::NodeHandle nh;
	vector<int> gear_ratios;
	vector<int> zero_points;

	int enc_resolution;

    nh.getParam("gear_ratios", gear_ratios);
    if( !nh.getParam("/gear_ratios", gear_ratios))
    	ROS_ERROR("Failed to get parameter from server.");

    //nh.getParam("zero_points", zero_points);
    //nh.getParam("enc_resolution", enc_resolution);
  	
  	
  	//int jnt_num = gear_ratios.size();
  	//cout << jnt_num << endl;
  	//Robot scorpio_arm(jnt_num, gear_ratios, zero_points, enc_resolution);

	return 0;
}