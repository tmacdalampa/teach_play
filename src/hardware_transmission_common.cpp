#include "teach_play/hardware_transmission_interface.h"
#include <fstream>
//#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

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
    jnt_num = 6;
    start_jnt = 0;
 	
    Init_Connection();
    EnableAll();
    ResetAll();

}

HwTmIntf::~HwTmIntf()
{

    DisableAll();
}

void HwTmIntf::Init_Connection()
{
    
    ELMO_INT32 iEventMask = 0x7FFFFFFF;
    const ELMO_PINT8 cHostIP= (char*)"192.168.1.7";
    const ELMO_PINT8 cDestIP= (char*)"192.168.1.3";

    // Set Try-Catch flag Enable\Disable
    CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
    CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

    //create connection
    gConnHndl = gConn.ConnectRPCEx(cHostIP, cDestIP, iEventMask, (MMC_MB_CLBK)NULL);
    gConn.GetVersion();

    // Register Run Time Error Callback function
    pRTEClbk = (RTE_CLBKP)OnRunTimeError;
    CMMCPPGlobal::Instance()->RegisterRTE(pRTEClbk);

    // Axes initialization
  	cAxis[0].InitAxisData("a01", gConnHndl);
    cAxis[1].InitAxisData("a02", gConnHndl);
  	cAxis[2].InitAxisData("a03", gConnHndl);
    cAxis[3].InitAxisData("a04", gConnHndl);
  	cAxis[4].InitAxisData("a05", gConnHndl);
  	cAxis[5].InitAxisData("a06", gConnHndl);
        
    cGrpRef.InitAxisData("v01",gConnHndl); //add this line and GG

}

void HwTmIntf::EnableAll()
{
    try
    { 
        for(int id = start_jnt; id < jnt_num; id++)
        {
            //ROS_INFO("axis %d status is %d (STAND_STILL) and %d (DISCRETE_MOTION)", id, (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK), (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK));
	    
            if (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[id].PowerOn();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
            //ROS_INFO("axis %d status is %d (STAND_STILL) and %d (DISCRETE_MOTION)", id, (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK), (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK));
        } 

        cGrpRef.GroupEnable();
        while (!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));

        //ROS_INFO("EnableAll Done");
        cout << "enable success" <<endl;
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
        for(int id = start_jnt; id < jnt_num; id++)
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
        cGrpRef.GroupDisable();
        while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));

        cout << "disable done" << endl;

    }
	catch(CMMCException exp)
  	{
        for(int id = start_jnt; id < jnt_num; id++)
      	{
            if (cAxis[id].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[id].Reset();
                while( !(cAxis[id].ReadStatus() & NC_AXIS_DISABLED_MASK));
            }
        }
 	}

}


void HwTmIntf::ResetAll()
{
	try
    {
        for(int id = start_jnt; id < jnt_num; id++)
        {
            if (cAxis[id].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[id].Reset();
                while( !(cAxis[id].ReadStatus() & NC_AXIS_DISABLED_MASK));
            }
        }
        
        if (cGrpRef.ReadStatus() & NC_GROUP_ERROR_STOP_MASK)
        {
            cGrpRef.GroupReset();
            while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
        }
		
        //ROS_INFO("ResetAll Done");
        std::cout << "reset all done" <<std::endl;
    }

    
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::ChangeOpMode(int op_mode)
{
    try
    {
        OPM402 eMode;

        switch(op_mode)
        {
            case 0: //position mode
                eMode = OPM402_CYCLIC_SYNC_POSITION_MODE;
            case 1:
                eMode = OPM402_CYCLIC_SYNC_TORQUE_MODE;
        }

        for(int id = start_jnt; id < jnt_num; id++)
        {
            
            if (cAxis[id].GetOpMode() != eMode)
            {
                cAxis[id].SetOpMode(eMode);
                while( cAxis[id].GetOpMode() != eMode);
            }
        }
        
        cout << "ChangeOpMode to Done" << endl;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}