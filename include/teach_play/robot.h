#include "teach_play/hardware_transmission_interface.h"
#include <array>

using namespace std;

class Robot
{
public:
	Robot(int jnt_num, vector<int> gear_ratios, vector<int> zero_points, int enc_resolution);
	~Robot();
private:
	
	vector<int> _gear_ratios;
	vector<int> _zero_points;
	int _enc_resolution;
	int _jnt_num;
};