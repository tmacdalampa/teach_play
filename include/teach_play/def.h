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
	PVT_NON_BLENDING = 0,
	PVT_BLENDING = 1,
	PVT_GO_STRAIGHT = 2
};

enum DIState
{
	BUTTON_PRESSED = 0,
	BUTTON_NON_PRESSED = 1
};