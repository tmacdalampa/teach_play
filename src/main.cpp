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

  Robot scorpio_arm(&nh);
  //HwTmIntf Master;
  
  #if 0
  while(ros::ok())
  {
    
    scorpio_arm.JointStatesPublisher();
  	scorpio_arm.RobotPosePublisher();
  	if (scorpio_arm.torque_mode_flag == true)
  	{
  		scorpio_arm.UpdateTorque();
  	}
  	
    cout << "hello" << endl;
  	ros::spinOnce();
  	rate.sleep();
  }
  #endif
  ros::spin();

	return 0;
}