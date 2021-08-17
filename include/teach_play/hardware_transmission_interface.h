#ifndef HARDEARE_TRANSMISSION_INTERFACE
#define HARDEARE_TRANSMISSION_INTERFACE

#include "teach_play/def.h"

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

#define JNT_NUM 6

class HwTmIntf
{
  public:
    HwTmIntf();
    ~HwTmIntf();

    void InitConnection();
    void EnableAll(bool op_mode);
    void DisableAll();
    void DisableGroup();
    void ResetAll(bool op_mode);
    void ChangeOpMode(bool op_mode, bool &torque_mode_ready_flag);
    void SelectModeProcess(bool op_mode, bool &torque_mode_ready_flag);
    void ReadENC(vector<double> &enc_cnts, vector<int> &vel_dir);
    void MoveTorque(vector<double> torque_cmd);


  private:
        
    //variable related with ELMO ethercat master
    MMC_CONNECT_HNDL gConnHndl ;
    CMMCConnection gConn ;
    CMMCSingleAxis cAxis[6];
    CMMCGroupAxis cGrpRef;
    CMMCHostComm cHost ;
    MMC_MOTIONPARAMS_SINGLE stSingleDefault ;
    RTE_CLBKP pRTEClbk;
    OPM402 eMode;

};

#endif
