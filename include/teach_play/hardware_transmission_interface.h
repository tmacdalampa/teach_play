#ifndef HARDEARE_TRANSMISSION_INTERFACE
#define HARDEARE_TRANSMISSION_INTERFACE

//include for C++
#include <iostream>
#include <sstream>
#include <vector>
#include "std_msgs/Bool.h"
#include <math.h>
#include <cmath>
#include <array>
#include <time.h>
//include for elmo
#include "MMCPP/OS_PlatformDependSetting.hpp"
#include "MMCPP/MMCPPlib.hpp"
#include "MMC_APP/MMC_definitions.h"





class HwTmIntf
{
  public:
    HwTmIntf();
    ~HwTmIntf();
    
    int start_jnt;
    int jnt_num;

    const std::vector<int> AxisGearRatios = {161, 161, 121, 161, 161, 101};
    const std::vector<int> ZeroPoints = {21075821, 3787515, -30318, -366005, 11787161, 40952066};
    const int ENC_FULL = 131072;
    const int acc_max = 100000000;

    void Init_Connection();
    void EnableAll();
    void DisableAll();
    void ResetAll();
    void ChangeOpMode(int op_mode);

  private:
    
    //variable for class
  	
    
    
    int point_num_real; //real point number put inside dTable
    
    double pvt_table_time_interval; // = time_final_real/point_num_real
    double vel_factor;
    
    std::vector<double> time_from_start_real;//time vector real
    std::vector<std::vector<int>> pvt_table; //final PVT vector synthesis by myself



    
    //variable related with ELMO ethercat master
    MMC_CONNECT_HNDL gConnHndl ;
    CMMCConnection gConn ;
    CMMCSingleAxis cAxis[6];
    CMMCGroupAxis cGrpRef;
    CMMCHostComm cHost ;
    MMC_MOTIONPARAMS_SINGLE stSingleDefault ;
    RTE_CLBKP pRTEClbk;

};

#endif
