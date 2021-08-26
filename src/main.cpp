//main.cpp
//include ros api and msgs
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

//include c++ stuff
#include <vector>
#include <string>
#include "teach_play/robot.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cobot_controller");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  /*
  HwTmIntf EcatMaster;
  while(ros::ok())
  {
    EcatMaster.ReadGroupStatus();
    rate.sleep();
  }
  */
  
  #if 1
  Robot scorpio_arm(&nh);
  while(ros::ok())
  {
    scorpio_arm.JointStatesPublisher();
    scorpio_arm.RobotPosePublisher();
    
    if (scorpio_arm.torque_mode_ready_flag == true)
  	{
      	scorpio_arm.UpdateTorque();
  	}
  	ros::spinOnce();
  	rate.sleep();
  }
  #endif

	return 0;
}