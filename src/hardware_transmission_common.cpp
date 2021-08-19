#include "teach_play/hardware_transmission_interface.h"
#include <fstream>
//#include <ros/ros.h>


#define PI 3.1415926

using namespace std;


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
    //ResetAll(false);
    //ChangeOpMode(false); //default position mode
    //EnableAll(false);

}

HwTmIntf::~HwTmIntf()
{
    OPM402 eMode_now = cAxis[5].GetOpMode();
    if (eMode_now == OPM402_CYCLIC_SYNC_POSITION_MODE)
    {
        DisableGroup();
    }
    DisableAll();

}

void HwTmIntf::InitConnection()
{
    
    ELMO_INT32 iEventMask = 0x7FFFFFFF;
    const ELMO_PINT8 cHostIP= (char*)"192.168.1.5";
    const ELMO_PINT8 cDestIP= (char*)"192.168.1.3";
    
    //cout << "A" << endl;
    // Set Try-Catch flag Enable\Disable
    CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
    CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);
    //cout << "B" << endl;
    //create connection
    gConnHndl = gConn.ConnectRPCEx(cHostIP, cDestIP, iEventMask, (MMC_MB_CLBK)NULL);
    gConn.GetVersion();
    //cout << "C" << endl;
    // Register Run Time Error Callback function
    pRTEClbk = (RTE_CLBKP)OnRunTimeError;
    CMMCPPGlobal::Instance()->RegisterRTE(pRTEClbk);
    //cout << "D" << endl;
    // Axes initialization
    
  	cAxis[0].InitAxisData("a01", gConnHndl);
    cAxis[1].InitAxisData("a02", gConnHndl);
  	cAxis[2].InitAxisData("a03", gConnHndl);
    cAxis[3].InitAxisData("a04", gConnHndl);
  	cAxis[4].InitAxisData("a05", gConnHndl);
  	cAxis[5].InitAxisData("a06", gConnHndl);   
    cGrpRef.InitAxisData("v01",gConnHndl); //add this line and GG
    
}

void HwTmIntf::SelectModeProcess(bool op_mode, bool &torque_mode_ready_flag)
{
    OPM402 eMode_now = cAxis[5].GetOpMode();
    if (op_mode == true)
    {
        if (eMode_now != OPM402_CYCLIC_SYNC_TORQUE_MODE)
        {
            //cout << "hello" << endl;
            DisableGroup();
            //DisableAll(op_mode);
            //ResetAll(op_mode);
            ChangeOpMode(op_mode, torque_mode_ready_flag);
        }
        EnableAll(op_mode);
        
        if (cAxis[5].GetOpMode() == OPM402_CYCLIC_SYNC_TORQUE_MODE)
        {
            torque_mode_ready_flag = true;
        }
    }
    else if (op_mode == false)
    {
        if (eMode_now != OPM402_CYCLIC_SYNC_POSITION_MODE)
        {
            //DisableAll(op_mode);
            //ResetAll(op_mode);
            ChangeOpMode(op_mode, torque_mode_ready_flag);
        }
        EnableAll(op_mode);
    }


}

void HwTmIntf::EnableAll(bool op_mode)
{
    try
    { 
        for(int id = 0; id < JNT_NUM; id++)
        {
            //ROS_INFO("axis %d status is %d (STAND_STILL) and %d (DISCRETE_MOTION)", id, (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK), (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK));
	    
            if (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[id].PowerOn();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
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

    }
    catch(CMMCException exp)
    {
        Exception(exp);
        cout << "enable failed" <<endl;
    }
}

void HwTmIntf::DisableAll()
{
    try
    {
        for(int id = 0; id < JNT_NUM; id++)
        {
            if (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[id].Stop();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
		    if (!(cAxis[id].ReadStatus()& NC_AXIS_DISABLED_MASK))
            {
                cAxis[id].PowerOff();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_DISABLED_MASK));
            }
        }
        


        cout << "disable done" << endl;

    }
	catch(CMMCException exp)
  	{
        Exception(exp);
 	}

}

void HwTmIntf::DisableGroup()
{
    try
    {   
        cGrpRef.GroupDisable();
        while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::ResetAll(bool op_mode)
{
	try
    {
        for(int id = 0; id < JNT_NUM; id++)
        {
            if (cAxis[id].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[id].Reset();
                while( !(cAxis[id].ReadStatus() & NC_AXIS_DISABLED_MASK));
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
    }

    
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::ChangeOpMode(bool op_mode, bool &torque_mode_ready_flag)
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

        for(int id = 0; id < JNT_NUM; id++)
        {            
            if (cAxis[id].GetOpMode() != eMode)
            {
                cAxis[id].SetOpMode(eMode);
                while( cAxis[id].GetOpMode() != eMode);
                
            }
            if (cAxis[id].GetOpMode() == OPM402_CYCLIC_SYNC_TORQUE_MODE)
            {
                torque_mode_ready_flag = true;
            }
        }
        cout << "ChangeOpMode Done" << endl;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::ReadENC(vector<double> &enc_cnts, vector<int> &vel_dir)
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
}


void HwTmIntf::MoveTorque(vector<double> torque_cmd)
{
    try
    {
        for (int i = 0; i < JNT_NUM; i++)
        {   
            cAxis[i].MoveTorque(torque_cmd[i], 10000, 100000, MC_ABORTING_MODE);
        }
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}


void HwTmIntf::PVTMotionMove(deque<vector<double>> &play_points)
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

    MC_PATH_REF handle;
    
    ELMO_DOUBLE dTable[NC_PVT_ECAM_MAX_ARRAY_SIZE]; 

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
            dTable[13*i] = 5;
      
            for(int j = 0; j<JNT_NUM; j++)
            {
                //time interval = 0
                dTable[13*i + 2*(j+1)] = 0; //vel = 0
                vector<double> six_axis_pt = play_points[i-1];
                dTable[13*i + 2*(j+1)-1] = six_axis_pt[j];
            }
        }
    }
    
    for(int i = 0; i<point_num + 1; i++)
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
        ulNumberOfPoints =point_num+1;
        ulStartIndex =0;
        ucIsAutoAppend =1;
        ucIsTimeAbsolute =0;
        eTableType = eNC_TABLE_PVT_ARRAY_QUINTIC_CUB;
        //cGrpRef.AppendPVTPoints(handle, dTable, ulNumberOfPoints,ulStartIndex,ucIsTimeAbsolute, eTableType); //manual
        cGrpRef.AppendPVTPoints(handle, dTable, ulNumberOfPoints, ucIsTimeAbsolute, eTableType); //auto
        std::cout << "AppendPVTPoints!!!" << std::endl;


        //Move PVT table
        cGrpRef.MovePVT(handle,eCoordSystem);
        std::cout << "Move!!!" << std::endl;


        //wait till motin ends
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        //return 0;

        cGrpRef.UnloadPVTTable(handle);
        std::cout << "UnloadPVTTable" << std::endl;
        while(!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));
        #endif

    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}
