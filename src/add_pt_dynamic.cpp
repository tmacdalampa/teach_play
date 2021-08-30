//main.cpp
//include ros api and msgs
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

//include c++ stuff
#include <vector>
#include <string>
#include "teach_play/hardware_transmission_interface.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_point_dynamic_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    double max_vel = 100000;
    deque<vector<double>> play_points;
    MotionType type = PVT_NON_BLENDING;
    vector<double> axis_deg1 = {529392, 475362, -1767978, -4064430, 19578637, 14497598};
    //vector<double> axis_deg1 = {0, 0, 0, 0, 0, 0};
    int point_num = 12;

    for(int i =0; i<=point_num; i++)
    {
        #if 0
        cout << axis_deg1[0] << " , "
            << axis_deg1[1] << " , " 
            << axis_deg1[2] << " , "
            << axis_deg1[3] << " , "
            << axis_deg1[4] << " , "
            << axis_deg1[5] << endl;
        #endif
        play_points.push_back(axis_deg1);
        int j = i/10;

        if (j % 2 == 0)axis_deg1[5] += 10000;
        else axis_deg1[5] -= 10000;    
    }

    play_points.clear();
    axis_deg1.clear();
  
  HwTmIntf EcatMaster;
  bool mode = false;
  bool mode_flag = false;

  EcatMaster.SelectModeProcess(mode, mode_flag); //select mode to cyclic position mode
  EcatMaster.AddPtDynamic(play_points);
  
  
  

	return 0;
}