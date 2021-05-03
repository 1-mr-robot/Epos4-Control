#include "ros/ros.h"
#include "epos4.h"
// #include <epos_arm_control/epos.h>

// epos_arm_control::epos arm_control;

// void moveArm(const epos_arm_control::epos::ConstPtr& params)
// {
// 	arm_control=*params;

// }



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
	

	ros::init(argc, argv, "epos_position");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	// ros::Subscriber sub = n.subscribe("exoskel_control", 1000, moveArm);

    while(ros::ok())
    {
    int position_new;
	get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);
    ros::spinOnce();
    }

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