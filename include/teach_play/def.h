#include <iostream>

enum VelDir
{
  POSITIVE = 1,
  NEGATIVE = -1,
  STANDSTILL = 0
};

enum DriverMode
{
	CST = 1,
	CSP = 0
};

enum MotionType
{
	GO_POINTS = 0,
	GO_STRAIGHT = 1,
	JOG = 2
};

enum DIState
{
	BUTTON_PRESSED = 0,
	BUTTON_NON_PRESSED = 1
};

enum GroupState
{
	MOTION = 1,
	STOP = 0
};

enum GetPoints
{
	ACS_ABSOLUTE = 0,
	ACS_RELATIVE = 1,
	MCS_ABSOLUTE = 2,
	MCS_RELATIVE = 3
};