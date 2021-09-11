#include "teach_play/hardware_transmission_interface.h"
#include <fstream>
//#include <ros/ros.h>


#define PI 3.1415926

using namespace std;
using namespace Eigen;


ELMO_INT32 OnRunTimeError(
    const ELMO_INT8* msg,
    MMC_CONNECT_HNDL uiConnHndl,
    ELMO_UINT16 usAxisRef,
    ELMO_INT16 sErrorID,
    ELMO_UINT16 usStatus)
{

    printf("\n  %s, Axis_ref=%d, ErrId=%d, status=%d \n", msg, usAxisRef, sErrorID, usStatus);
    MMC_CloseConnection(uiConnHndl);

    exit(0);
}


void Exception(CMMCException exp)
{
    std::cout << "function " << exp.what() << std::endl;
    std::cout << "axis " << exp.axisRef() << " error: " << exp.error() << std::endl;
    exit(0);
}


HwTmIntf::HwTmIntf()
{    
    InitConnection();
    
    if (ResetAll() != true)
    {
        exit(0);
    }

    if(EnableDrivers() != true)
    {
        exit(0);
    }
}

HwTmIntf::~HwTmIntf()
{
    
    DisableGroup();
    DisableDrivers();
        

}

void HwTmIntf::InitConnection()
{
    
    ELMO_INT32 iEventMask = 0x7FFFFFFF;
    //const ELMO_PINT8 cHostIP= (char*)"192.168.1.7";
    //const ELMO_PINT8 cDestIP= (char*)"192.168.1.3";
    const ELMO_PINT8 cHostIP= (char*)"10.42.0.2";
    const ELMO_PINT8 cDestIP= (char*)"10.42.0.14";
    
    CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
    CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

    gConnHndl = gConn.ConnectRPCEx(cHostIP, cDestIP, iEventMask, (MMC_MB_CLBK)NULL);
    gConn.GetVersion();

    pRTEClbk = (RTE_CLBKP)OnRunTimeError;
    CMMCPPGlobal::Instance()->RegisterRTE(pRTEClbk);

    
  	cAxis[0].InitAxisData("a01", gConnHndl);
    cAxis[1].InitAxisData("a02", gConnHndl);
  	cAxis[2].InitAxisData("a03", gConnHndl);
    cAxis[3].InitAxisData("a04", gConnHndl);
  	cAxis[4].InitAxisData("a05", gConnHndl);
  	cAxis[5].InitAxisData("a06", gConnHndl);   
    cGrpRef.InitAxisData("v01",gConnHndl);
    
}

bool HwTmIntf::SelectModeProcess(bool op_mode, bool &torque_mode_ready_flag)
{
    DriverMode driver_mode = GetDriverMode();
    if (op_mode == true)
    {
        if(driver_mode != DriverMode::CST)        
        {
            if (DisableGroup() != true) return false; 
            if (ChangeOpMode(op_mode, torque_mode_ready_flag) != true) return false;
        }
    }
    else if (op_mode == false)
    {
        if(driver_mode != DriverMode::CSP)        
        {
            if (ChangeOpMode(op_mode, torque_mode_ready_flag) != true) return false;
        }
        if (EnableGroup() != true) return false;
    }

    return true;
}

bool HwTmIntf::EnableDrivers()
{
    try
    { 
        for(int i = 0; i < JNT_NUM; i++)
        {
            if (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[i].PowerOn();
                while (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
        }

        return true;

    }
    catch(CMMCException exp)
    {
        Exception(exp);
        cout << "enable failed" <<endl;
        return false;
    }
}

bool HwTmIntf::EnableGroup()
{
    try
    {
        cGrpRef.GroupEnable();
        while (!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}

bool HwTmIntf::DisableDrivers()
{
    try
    {
        for(int i = 0; i < JNT_NUM; i++)
        {
		    if (!(cAxis[i].ReadStatus()& NC_AXIS_DISABLED_MASK))
            {
                cAxis[i].PowerOff();
                while (!(cAxis[i].ReadStatus()& NC_AXIS_DISABLED_MASK));
            }
        }
        return true;
    }
	catch(CMMCException exp)
  	{
        Exception(exp);
        return false;
 	}

}

bool HwTmIntf::DisableGroup()
{
    try
    {   
        if (!(cGrpRef.ReadStatus()& NC_AXIS_DISABLED_MASK))
        {    
            cGrpRef.GroupDisable();
        }
        while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}

bool HwTmIntf::ResetAll()
{
	try
    {
        for(int i = 0; i < JNT_NUM; i++)
        {
            if (cAxis[i].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[i].Reset();
                while( !(cAxis[i].ReadStatus() & NC_AXIS_DISABLED_MASK));
            }
        }

		
        if (cGrpRef.ReadStatus() & NC_GROUP_ERROR_STOP_MASK)
        {
            cGrpRef.GroupReset();
            while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
        }
        
        
        cout << "reset all done" << endl;
        return true;
    }

    
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}

bool HwTmIntf::ChangeOpMode(bool op_mode, bool &torque_mode_ready_flag)
{
    try
    {
        if (op_mode == true)
        {
            eMode = OPM402_CYCLIC_SYNC_TORQUE_MODE;
        }
        else
        {
            eMode = OPM402_CYCLIC_SYNC_POSITION_MODE;
            torque_mode_ready_flag = false;
        }

        for(int i = 0; i < JNT_NUM; i++)
        {            
            if (cAxis[i].GetOpMode() != eMode)
            {
                cAxis[i].SetOpMode(eMode);
                while( cAxis[i].GetOpMode() != eMode);
                
            }
            if (cAxis[i].GetOpMode() == OPM402_CYCLIC_SYNC_TORQUE_MODE)
            {
                torque_mode_ready_flag = true;
            }
        }
        cout << "ChangeOpMode success" << endl;
        return true;
    }
    catch(CMMCException exp)
    {
        cout << "ChangeOpMode failed" << endl;
        Exception(exp);
        return false;
    }
}

DriverMode HwTmIntf::GetDriverMode()
{
    try
    {
        OPM402 eMode_now;
        for(int i = 0; i < JNT_NUM; i++)
        {    
            eMode_now =  cAxis[i].GetOpMode();
        }
        if (eMode_now == OPM402_CYCLIC_SYNC_TORQUE_MODE)
        {
            return DriverMode::CST;
        }
        else
        {
            return DriverMode::CSP;
        }
    }

    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

bool HwTmIntf::ReadENC(vector<double> &enc_cnts, vector<int> &vel_dir)
{
    try
    {
        for (int i = 0; i < JNT_NUM; i++)
        {
            enc_cnts[i] = cAxis[i].GetActualPosition();
        
            if(cAxis[i].GetActualVelocity() > 1000)
            {   
                vel_dir[i] = VelDir::POSITIVE;
            }
            else if(cAxis[i].GetActualVelocity() < -1000)
            {
                vel_dir[i] = VelDir::NEGATIVE;
            }
            else
            {
                vel_dir[i] = VelDir::STANDSTILL;
            }    
        }
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}


bool HwTmIntf::MoveTorque(vector<double> torque_cmd)
{
    try
    {
        for (int i = 0; i < JNT_NUM; i++)
        {   
            cAxis[i].MoveTorque(torque_cmd[i], 10000, 100000, MC_ABORTING_MODE);
        }
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}

#if 0
bool HwTmIntf::PVTMotionMove(deque<vector<double>> &play_points, double &max_vel, MotionType &types)
{
    ELMO_INT16 iCnt;

    //PVT init data
    ELMO_ULINT32 ulMaxPoints;
    ELMO_ULINT32 ulUnderflowThreshold ;
    ELMO_UINT8 ucIsCyclic;
    ELMO_UINT8 ucIsPosAbsolute;
    ELMO_UINT16 usDimension;
    MC_COORD_SYSTEM_ENUM eCoordSystem;
    NC_MOTION_TABLE_TYPE_ENUM eTableMode;

    //PVT appaend data
    ELMO_ULINT32 ulNumberOfPoints;
    ELMO_ULINT32 ulStartIndex;
    ELMO_UINT8 ucIsAutoAppend;
    ELMO_UINT8 ucIsTimeAbsolute;
    NC_MOTION_TABLE_TYPE_ENUM eTableType = eNC_TABLE_PVT_ARRAY_QUINTIC_CUB;

    
    
     

    //MotionPlanning(play_points, max_velm, motion_types);

   
    int total_pt_num;//
    switch(types)
    { 
        case MotionType::PVT_NON_BLENDING:
            total_pt_num = MotionPlanningNonBlending(play_points, max_vel);           
            break;
        case MotionType::PVT_BLENDING:
            total_pt_num = MotionPlanningBlending(play_points, max_vel);      
            break;

    } 
    
    #if 1
    for(int i = 0; i < total_pt_num; i++)
    {
        cout << dTable[13*i] << " , "
       << dTable[13*i+1] << " , "
       << dTable[13*i+2] << " , "
       << dTable[13*i+3] << " , "
       << dTable[13*i+4] << " , "
       << dTable[13*i+5] << " , "
       << dTable[13*i+6] << " , "
       << dTable[13*i+7] << " , "
       << dTable[13*i+8] << " , "
       << dTable[13*i+9] << " , "
       << dTable[13*i+10] << " , "
       << dTable[13*i+11] << " , "
       << dTable[13*i+12] << " , " << endl;
    }
    #endif

    try
    {

        #if 1
        //Init PVT memory table
        ulMaxPoints= 13;
        ulUnderflowThreshold =3 ;
        ucIsCyclic = 0;
        ucIsPosAbsolute =1;
        usDimension =6;
        eCoordSystem = MC_ACS_COORD;
        eTableMode  = eNC_TABLE_PVT_ARRAY_QUINTIC_CUB;

        handle = cGrpRef.InitPVTTable(  ulMaxPoints,
                                        ulUnderflowThreshold,
                                        ucIsCyclic,
                                        ucIsPosAbsolute,
                                        usDimension,
                                        eCoordSystem,
                                        eTableMode);
        std::cout << "InitPVTTable!!!" << std::endl;



        //Append PVT memory table points
        ulNumberOfPoints =total_pt_num;
        ulStartIndex = 0;
        ucIsAutoAppend =1;
        ucIsTimeAbsolute =0;
        eTableType = eNC_TABLE_PVT_ARRAY_QUINTIC_CUB;
        cGrpRef.AppendPVTPoints(handle, dTable, ulNumberOfPoints, ucIsTimeAbsolute, eTableType); //auto
        std::cout << "AppendPVTPoints!!!" << std::endl;
        //int haha = MMC_GetTableIndex();

        //Move PVT table
        cGrpRef.MovePVT(handle,eCoordSystem);
        std::cout << "Move!!!" << std::endl;

        /*
        //wait till motin ends
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        //return 0;

        cGrpRef.UnloadPVTTable(handle);
        std::cout << "UnloadPVTTable" << std::endl;
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        */
        #endif
        return true;

    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
}


int HwTmIntf::MotionPlanningNonBlending(deque<vector<double>> &play_points, double &max_vel)
{
    int point_num = play_points.size();
    for(int i = 0; i < point_num +1; i++)
    {
        if (i == 0)
        {
            dTable[13*i] = 0; //t0=0
      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0
                vector<double> six_axis_pt = play_points.back();
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
            }
        }
        else
        {
            vector<double> path_each_joint;
            vector<double> six_axis_pt = play_points[i-1];      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0    
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
                path_each_joint.push_back(abs(dTable[13*i + 2*(j+1)-1] - dTable[13*(i-1) + 2*(j+1)-1]));
            }
            double max_path = *max_element(path_each_joint.begin(), path_each_joint.end());

            dTable[13*i] = max_path/max_vel;
            cout << "this path time interval = " << max_path/max_vel << endl;
        }
    }
    return point_num+1;
}

int HwTmIntf::MotionPlanningBlending(deque<vector<double>> &play_points, double &max_vel)
{
    int point_num = play_points.size();
    for(int i = 0; i < point_num +1; i++)
    {
        if (i == 0)
        {
            dTable[13*i] = 0; //t0=0
      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0
                vector<double> six_axis_pt = play_points.back();
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
            }
        }
        else
        {
            vector<double> path_each_joint;
            vector<double> six_axis_pt = play_points[i-1];      
            for(int j = 0; j<JNT_NUM; j++)
            {
                dTable[13*i + 2*(j+1)] = 0; //vel = 0    
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
                path_each_joint.push_back(abs(dTable[13*i + 2*(j+1)-1] - dTable[13*(i-1) + 2*(j+1)-1]));
            }
            double max_path = *max_element(path_each_joint.begin(), path_each_joint.end());

            dTable[13*i] = max_path/max_vel;
            cout << "this path time interval = " << max_path/max_vel << endl;
        }
    }
    
    MatrixXd Axis1_B_Matrix(point_num-1, 1);
    MatrixXd Axis2_B_Matrix(point_num-1, 1);
    MatrixXd Axis3_B_Matrix(point_num-1, 1);
    MatrixXd Axis4_B_Matrix(point_num-1, 1);
    MatrixXd Axis5_B_Matrix(point_num-1, 1);
    MatrixXd Axis6_B_Matrix(point_num-1, 1);
    MatrixXd Time_Matrix(point_num-1, point_num-1);

    for(int i = 0;i <= point_num - 2;i++)
    {
        for(int j = 0;j <= point_num - 2;j++)
        {
            Time_Matrix(i,j) = 0;
        }
        Axis1_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+1] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+1] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+1]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]);
        
        Axis2_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+3] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+3] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+3]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis3_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+5] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+5] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+5]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis4_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+7] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+7] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+7]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis5_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+9] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+9] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+9]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
        
        Axis6_B_Matrix(i, 0) = (-3 * pow(dTable[13*(i + 2)],2) * dTable[13*i+11] + 3 * (pow(dTable[13*(i + 2)], 2) - pow(dTable[13*(i + 1)], 2)) * dTable[13*(i+1)+11] + 
        3 * pow(dTable[13*(i+1)],2) * dTable[13*(i+2)+11]) / (dTable[13*(i + 2)] * dTable[13*(i + 1)]); 
    }

    for (int i  = 0; i <= point_num-2; i++)
    {
        Time_Matrix(i, i) = 2 * (dTable[13*(i + 2)] + dTable[13*(i + 1)]);

        if (i > 0 && i < point_num - 2)
        {
            Time_Matrix(i, i - 1) = dTable[13*(i + 2)];
            Time_Matrix(i, i + 1) = dTable[13*(i + 1)];
        }
        else if (i == 0)
        {
            Time_Matrix(i, i + 1) = dTable[13*(i + 1)];
        }
        else if (i == point_num-2)
        {
            Time_Matrix(i, i - 1) = dTable[13*(i + 2)];
        }
    }

    MatrixXd Axis1_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis2_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis3_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis4_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis5_Vel_Matrix(point_num-1, 1);
    MatrixXd Axis6_Vel_Matrix(point_num-1, 1);


    Axis1_Vel_Matrix = Time_Matrix.inverse() * Axis1_B_Matrix;
    Axis2_Vel_Matrix = Time_Matrix.inverse() * Axis2_B_Matrix;
    Axis3_Vel_Matrix = Time_Matrix.inverse() * Axis3_B_Matrix;
    Axis4_Vel_Matrix = Time_Matrix.inverse() * Axis4_B_Matrix;
    Axis5_Vel_Matrix = Time_Matrix.inverse() * Axis5_B_Matrix;
    Axis6_Vel_Matrix = Time_Matrix.inverse() * Axis6_B_Matrix;

    for(int i = 0; i<point_num-1; i++)
    {
        dTable[13*(i+1) + 2] = Axis1_Vel_Matrix(i,0);
        dTable[13*(i+1) + 4] = Axis2_Vel_Matrix(i,0);
        dTable[13*(i+1) + 6] = Axis3_Vel_Matrix(i,0);
        dTable[13*(i+1) + 8] = Axis4_Vel_Matrix(i,0);
        dTable[13*(i+1) + 10] = Axis5_Vel_Matrix(i,0);
        dTable[13*(i+1) + 12] = Axis6_Vel_Matrix(i,0);
    }

    return point_num+1;
}

int HwTmIntf::MotionPlanningGoStraight(deque<vector<double>> &play_points, double &max_vel)
{
    
    int point_num = play_points.size();

    for(int i = 0; i < point_num; i++)
    {
        vector<double> path_each_joint;
        vector<double> six_axis_pt = play_points[i];      
        for(int j = 0; j<JNT_NUM; j++)
        {
            dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
            path_each_joint.push_back(abs(dTable[13*i + 2*(j+1)-1] - dTable[13*(i-1) + 2*(j+1)-1]));
        }
        double max_path = *max_element(path_each_joint.begin(), path_each_joint.end());

        dTable[13*i] = max_path/max_vel;

        cout << "this path time interval = " << max_path/max_vel << endl;
    }
    dTable[0] = 0;
    
    return point_num;

}
#endif

DIState HwTmIntf::GetDISignal(int digital_input_number)
{
    try
    {
        ELMO_UINT8 res;
        res = cAxis[5].GetDigInput(digital_input_number);
        //digital_input_number = 16, left button
        //digital_input_number = 17, right button
        DIState state;
        switch(res)
        { 
            case 1:
                state = BUTTON_NON_PRESSED;           
                break;
            case 0:
                state = BUTTON_PRESSED;      
                break;
        }
        return state;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

bool HwTmIntf::StopMotion()
{   try
    {   
        if(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK))
        {
            cout << "stop" << endl;
            cGrpRef.GroupHalt(1000000, 10000000, MC_ABORTING_MODE);
            while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        }
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

GroupState HwTmIntf::CheckGroupStatus()
{
    if(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK))
        return GroupState::MOTION;//still moving
    else
        return GroupState::STOP;
    
}

void HwTmIntf::UnloadTable()
{
    cGrpRef.UnloadPVTTable(handle);
    cout << "UnloadPVTTable" << endl;
    while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
}


int HwTmIntf::GetTableIndex()
{
    int index = cGrpRef.GetPVTTableIndex(handle);
    return index;
}

bool HwTmIntf::SetSpeedOverride(double vel_factor)
{
    try
    {
        cGrpRef.GroupSetOverride(vel_factor, vel_factor, vel_factor, 0);
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

bool HwTmIntf::GroupLinearMotionMove(deque<vector<double>> &points, double &max_vel)
{
    try
    {
        G_Mparam.eBufferMode = MC_BUFFERED_MODE;
        G_Mparam.eCoordSystem = MC_ACS_COORD;
        G_Mparam.eTransitionMode = MC_TM_CORNER_DIST_CV_POLYNOM5_NAXES;
        G_Mparam.fVelocity = max_vel;         //TODO
        G_Mparam.fAcceleration = 10*max_vel;    //TODO
        G_Mparam.fDeceleration = 10*max_vel;    //TODO
        G_Mparam.fJerk = 100*max_vel;

        cGrpRef.SetDefaultParams(G_Mparam);
        
        int point_num = points.size();
        
        for(int i = 0; i < point_num; i++)
        {
            vector<double> goal_position = points[i];
            for (int j = 0; j < JNT_NUM; j++)
            {
                G_Mparam.dEndPoint[j] = goal_position[j];
            }

            cGrpRef.MoveLinearAbsolute(G_Mparam.fVelocity, G_Mparam.dEndPoint, G_Mparam.fAcceleration, G_Mparam.fDeceleration, G_Mparam.fJerk, G_Mparam.eBufferMode);
        }
        return true;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}
