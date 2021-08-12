#include "teach_play/robot.h"

using namespace std;

Robot::Robot(int jnt_num, vector<int> gear_ratios, vector<int> zero_points, int enc_resolution)
{
	#if 0
	for(int i =0; i < 6; i++)
	{
		_gear_ratios.push_back(gear_ratios[i]);
		_zero_points.push_back(zero_points[i]);
		
	}
	#endif
	_gear_ratios = gear_ratios;
	_zero_points = zero_points;
	_enc_resolution = enc_resolution;
	_jnt_num = jnt_num;
	cout << _jnt_num << endl;
	/*for(int i =0; i < 6; i++)
	{
		cout << _gear_ratios[i] << endl;
	}
	*/
}
Robot::~Robot()
{

}