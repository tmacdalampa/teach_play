# teach_play
this is a robot arm teach and play function under ROS environment

teach_play_state_machine.py:
This is the state machine that user can type simple command in the command line to interact with the arm.

LaserManager.py:
This node is a subscriber of Laser Data, running this node and then the robot arm will slow down or stop during moving if somebody bump into its protection zone.

robot.cpp:
This class included function like IK, FK, gravity torque and some interface of ROS.

hardware_transmission_common.cpp"
This class is composed of ELMO Platinum Motion Controller Linux Library, one can re-write this file by different hardware.

How to run:

main code:
roslaunch teach_play teach_play.launch

state machine:
rosrun teach_play teach_play_state_machine.py

Protection Zone:
rosrun teach_play LaserManager.py
