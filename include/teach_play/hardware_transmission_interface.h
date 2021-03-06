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
#include <deque>
#include <utility>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

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
    bool EnableDrivers();
    bool EnableGroup();
    bool DisableDrivers();
    bool DisableGroup();
    bool ResetAll();
    bool ChangeOpMode(bool op_mode, bool &torque_mode_ready_flag);
    bool SelectModeProcess(bool op_mode, bool &torque_mode_ready_flag);
    bool ReadENC(vector<double> &enc_cnts, vector<int> &vel_dir);
    bool MoveTorque(vector<double> torque_cmd);
    //bool PVTMotionMove(deque<vector<double>> &play_points, double &max_vel, MotionType &types);
    DriverMode GetDriverMode();
    DIState GetDISignal(int digital_input_number);
    bool StopMotion();
    GroupState CheckGroupStatus();
    void UnloadTable();
    int GetTableIndex();
    bool SetSpeedOverride(double vel_factor);
    bool GroupLinearMotionMove(deque<vector<double>> &points, double &max_vel);



  private:
    //int MotionPlanningNonBlending(deque<vector<double>> &play_points, double &max_vel);
    //int MotionPlanningBlending(deque<vector<double>> &play_points, double &max_vel);
    //int MotionPlanningGoStraight(deque<vector<double>> &play_points, double &max_vel);

    //variable related with ELMO ethercat master
    MMC_CONNECT_HNDL gConnHndl ;
    CMMCConnection gConn ;
    CMMCSingleAxis cAxis[6];
    CMMCGroupAxis cGrpRef;
    CMMCHostComm cHost ;
    MMC_MOTIONPARAMS_SINGLE stSingleDefault ;
    RTE_CLBKP pRTEClbk;
    OPM402 eMode;
    ELMO_DOUBLE dTable[NC_PVT_ECAM_MAX_ARRAY_SIZE];

    MMC_MOTIONPARAMS_GROUP G_Mparam;

    MC_PATH_REF handle;


};

#endif
