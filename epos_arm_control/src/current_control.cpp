#include "ros/ros.h"
#include "epos4.h"
#include <epos_arm_control/epos.h>

epos_arm_control::epos arm_control;

void moveArm(const epos_arm_control::epos::ConstPtr& params)
{
	arm_control=*params;
    unsigned int ulErrorCode = 0;
	stringstream msg;

	msg << "set profile current mode, node = " << g_usNodeId<<"\n";
	LogInfo(msg.str());

    msg << "move with torque = " << (long)arm_control.torque << ", node = " << g_usNodeId<<"\n";
	LogInfo(msg.str());
    
    SetCurrentMust(g_pKeyHandle,g_usNodeId,(short)arm_control.current,&ulErrorCode);

    short current;
	int position_new,current_velocity;
    get_velocity(g_pKeyHandle,g_usNodeId,&current_velocity,&ulErrorCode);

    get_current(g_pKeyHandle, g_usNodeId, &current, &ulErrorCode);

    // res.current = current;
	// res.position_new = position_new;
	// res.velocity = pVelocityIs;

	get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);
	//HaltPosition(g_pKeyHandle,g_usNodeId);
    //HaltVelocity(g_pKeyHandle,g_usNodeId);

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
	
	if((lResult = ActivateProfileCurrentMode(g_pKeyHandle,g_usNodeId,&ulErrorCode))!=MMC_SUCCESS)
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