#include "ros/ros.h"
#include "epos4.h"
#include <epos_arm_control/epos.h>

epos_arm_control::epos arm_control;

void moveArm(const epos_arm_control::epos::ConstPtr& msg)
{
	arm_control=*msg;
    unsigned int ulErrorCode = 0;
	stringstream msgs;
	int absolute=1;
	int relative=0;

	msgs << "set profile position mode, node = " << g_usNodeId;
	LogInfo(msgs.str());

    msgs << "move to position = " << (long)arm_control.position << ", node = " << g_usNodeId;
	LogInfo(msgs.str());
    
    MoveToPosition(g_pKeyHandle, g_usNodeId, (long)arm_control.position,relative, &ulErrorCode);
	short current;
	int position_new;
	get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);
	// res.current = current;
	msgs << "new position = " << position_new << ", node = " << g_usNodeId;
	LogInfo(msgs.str());
	//HaltPosition(g_pKeyHandle,g_usNodeId);

}



int main(int argc, char **argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

    // Print maxon headerline
    PrintHeader();

	// Set parameter for usb IO operation
	SetDefaultParameters();

    //Print default parameters
    PrintSettings();


	// open device
	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

    // set enable state
	if((lResult = SetEnableState(g_pKeyHandle,g_usNodeId,&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("EnableState", lResult, ulErrorCode);
		return lResult;
	}
	
	if((lResult = ActivateProfilePositionMode(g_pKeyHandle,g_usNodeId,&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("Activate Mode", lResult, ulErrorCode);
		return lResult;
	}

	ros::init(argc, argv, "epos_position");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);
	ros::Subscriber sub = n.subscribe("exoskel_control", 1000, moveArm);

	ros::spin();

	//disable epos
	SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	//close device
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return 0;

}