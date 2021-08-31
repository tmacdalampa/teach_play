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
    //int tmp = cGrpRef.GetPVTTableIndex(handle);
    //cout << "current talbe index" << tmp << endl;
    
    if (ResetAll(true) != true)
    {
        exit(0);
    }
}

HwTmIntf::~HwTmIntf()
{
    _res = DisableAll();

    DriverMode mode = GetDriverMode();

    if (mode == CSP)
    {
        _res = DisableGroup();
    }
    

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
    OPM402 eMode_now = cAxis[5].GetOpMode();
    if (op_mode == true)
    {
        if (eMode_now != OPM402_CYCLIC_SYNC_TORQUE_MODE)
        {
            if (DisableGroup() != true) return false; 
            if (ChangeOpMode(op_mode, torque_mode_ready_flag) != true) return false;
        }
        
        if ( EnableAll(op_mode)!= true) return false;
        
        if (cAxis[5].GetOpMode() == OPM402_CYCLIC_SYNC_TORQUE_MODE)
        {
            torque_mode_ready_flag = true;
        }
    }
    else if (op_mode == false)
    {
        if (eMode_now != OPM402_CYCLIC_SYNC_POSITION_MODE)
        {
            if (ChangeOpMode(op_mode, torque_mode_ready_flag) != true) return false;
        }
        if (EnableAll(op_mode) != true) return false;
    }

    return true;
}

bool HwTmIntf::EnableAll(bool op_mode)
{
    try
    { 
        for(int i = 0; i < JNT_NUM; i++)
        {
            //ROS_INFO("axis %d status is %d (STAND_STILL) and %d (DISCRETE_MOTION)", id, (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK), (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK));
	    
            if (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[i].PowerOn();
                while (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
            //ROS_INFO("axis %d status is %d (STAND_STILL) and %d (DISCRETE_MOTION)", id, (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK), (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK));
        }

        //ROS_INFO("EnableAll Done");
        cout << "enable success" <<endl;
        if (op_mode == false)
        {
            cGrpRef.GroupEnable();
            while (!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
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

bool HwTmIntf::DisableAll()
{
    try
    {
        for(int i = 0; i < JNT_NUM; i++)
        {
            //if (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            //{
            //    cAxis[i].Stop(); //if it is still moving, start first
            //    while (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            //}
		    if (!(cAxis[i].ReadStatus()& NC_AXIS_DISABLED_MASK))
            {
                cAxis[i].PowerOff();
                while (!(cAxis[i].ReadStatus()& NC_AXIS_DISABLED_MASK));
            }
        }
        cout << "disable done" << endl;
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
        
        cout << "disable group success" << endl;
        return true;
    }
    catch(CMMCException exp)
    {
        cout << "disable group failed" << endl;
        Exception(exp);
        return false;
    }
}

bool HwTmIntf::ResetAll(bool op_mode)
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
        if(op_mode == false)
		{
            if (cGrpRef.ReadStatus() & NC_GROUP_ERROR_STOP_MASK)
            {
                cGrpRef.GroupReset();
                while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
            }
        }
        //ROS_INFO("ResetAll Done");
        std::cout << "reset all done" <<std::endl;
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
        case MotionType::PVT_GO_STRAIGHT:
            total_pt_num = MotionPlanningGoStraight(play_points, max_vel);  
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
            cGrpRef.GroupHalt(1000000, 10000000, MC_ABORTING_MODE);
        }
        
        /*
        _res = DisableGroup();
        for(int i = 0; i < JNT_NUM; i++)
        {
            if (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[i].Stop(); //if it is still moving, start first
                while (!(cAxis[i].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
        }
        */
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

bool HwTmIntf::AddPtDynamic(deque<vector<double>> &points)
{
    /*cGrpRef.UnloadPVTTable(handle);
    std::cout << "UnloadPVTTable" << std::endl;
    while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
    */
    
	
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
    
    int total_pt_num = points.size();
    cout << "total_pt_num = " << total_pt_num << endl;

    for(int i = 0; i < 12; i++)
    {
        vector<double> six_axis_pt = points.front();      
        for(int j = 0; j<JNT_NUM; j++)
        {
            dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
        }
        
        dTable[13*i] = 1;
        if (i == 0) dTable[i] = 0;
        points.pop_front();
    }
    
    /*
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
    */
    
    try
    {
        //Init PVT memory table
        ulMaxPoints= 16;
        ulUnderflowThreshold = 1;
        ucIsCyclic = 0;
        ucIsPosAbsolute =1;
        usDimension = 6;
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
        ulNumberOfPoints = 12;
        ulStartIndex = 0;
        ucIsAutoAppend = 0;
        ucIsTimeAbsolute = 0;
        eTableType = eNC_TABLE_PVT_ARRAY_QUINTIC_CUB;
        //cGrpRef.AppendPVTPoints(handle, dTable, ulNumberOfPoints, ucIsTimeAbsolute, eTableType); //auto
        cGrpRef.AppendPVTPoints(handle, dTable, ulNumberOfPoints, ulStartIndex ,ucIsTimeAbsolute, eTableType); //auto
        std::cout << "AppendPVTPoints!!!" << std::endl;

        //ELMO_INT32 num;
        //int tmp = cGrpRef.GetTableList(eTableType, handle, num);

        //Move PVT table
        cGrpRef.MovePVT(handle,eCoordSystem);
        std::cout << "Move!!!" << std::endl;
        int index_pre = 1000;
        bool append_1_flag = true;
        bool append_2_flag = false;
        
        //wait till motin ends
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK))
        {
        	//int index = cGrpRef.GetPVTTableIndex(handle);
        	//cout << "pvt table index = " << index << endl;

        	
            int index = cGrpRef.GetPVTTableIndex(handle);
            if (index != index_pre) cout << "pvt table index = " << index << endl;
            index_pre = index;
            #if 0        	
        	if (points.empty() != true)
        	{        		
        		if (index > 5 && append_1_flag == true)
        		{
        			if (points.size() >= 6)
        			{
        				for(int i = 0; i < 6; i++)
    					{
        					vector<double> path_each_joint;
        					vector<double> six_axis_pt = points.front();      
        					for(int j = 0; j<JNT_NUM; j++)
        					{
            					dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            					dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
        					}
        			        		
        					dTable[13*i] = 1;
        					points.pop_front();
    					}
                        cout << "a" << endl;
                        for(int i = 0; i < 12; i++)
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
    					cGrpRef.AppendPVTPoints(handle, dTable, 6, ulStartIndex, ucIsTimeAbsolute, eTableType);
                        append_1_flag = false;
        			    append_2_flag = true;
                        
                        
    				}
        			else
        			{
        				int last_size = points.size();
        				for(int i = 0; i < last_size; i++)
    					{
        					vector<double> path_each_joint;
        					vector<double> six_axis_pt = points.front();      
        					for(int j = 0; j<JNT_NUM; j++)
        					{
            					dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            					dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
        					}
        					
        					dTable[13*i] = 1;
        					points.pop_front();
    					}
                        cout << "b" << endl;	
    					cGrpRef.AppendPVTPoints(handle, dTable, last_size, ulStartIndex, ucIsTimeAbsolute, eTableType);
                        append_1_flag = false;
        			    append_2_flag = true;
        			}

        			
        		}
        		else if(index <= 5 && append_2_flag == true)
        		{
        			if (points.size() >= 6)
        			{
        				for(int i = 6; i < 12; i++)
    					{
        					vector<double> path_each_joint;
        					vector<double> six_axis_pt = points.front();      
        					for(int j = 0; j<JNT_NUM; j++)
        					{
            					dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            					dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
        					}
        					
        					dTable[13*i] = 1;
        					points.pop_front();
                            
    					}
                        cout << "c" << endl;
                        for(int i = 0; i < 12; i++)
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
    					cGrpRef.AppendPVTPoints(handle, dTable, 6, 6,ucIsTimeAbsolute, eTableType);
                        append_1_flag = false;
        		        append_2_flag = true;
                        
    				}
        			else
        			{
        				int last_size = points.size();
        				for(int i = 6; i < 6+last_size; i++)
    					{
        					vector<double> path_each_joint;
        					vector<double> six_axis_pt = points.front();      
        					for(int j = 0; j<JNT_NUM; j++)
        					{
            					dTable[13*i + 2*(j+1)] = 0; //vel = 0    
            					dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
        					}
        					
        					dTable[13*i] = 1;
        					points.pop_front();
    					}
                        cout << "d" << endl;
    					cGrpRef.AppendPVTPoints(handle, dTable, last_size, 6,ucIsTimeAbsolute, eTableType);
        				//append second part
                        append_1_flag = false;
        			    append_2_flag = true;
        			}
                    
                    
        		}
        	}

        	#endif
        }
        

        cGrpRef.UnloadPVTTable(handle);
        std::cout << "UnloadPVTTable" << std::endl;
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));

        return true;

    }
    catch(CMMCException exp)
    {
        Exception(exp);
        return false;
    }
    
}

int HwTmIntf::GetTableIndex()
{
    int index = cGrpRef.GetPVTTableIndex(handle);
    return index;
}
