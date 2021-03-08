#include "ros/ros.h"
#include "epos4.h"
#include <epos_arm_control/epos.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

epos_arm_control::epos arm_control;
std_msgs::Float64MultiArray angle;
std_msgs::Float64 error;



void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
	angle=*array;

    // for(int j = 0; j < 3; j++)
	// {	
	// 	printf("%lf, ", angle.data[j]);
	// }


	// printf("\n");

}


void moveArm(const epos_arm_control::epos::ConstPtr& params)
{
	arm_control=*params;
    // unsigned int ulErrorCode = 0;
	// int absolute=1;
	// int relative=0;
	//int i;
	//long error_inc;

	// msg << "set profile position mode, node = " << g_usNodeId<<"\n";
	// LogInfo(msg.str());

    // msg << "move to position = " << (long)arm_control.position << ", node = " << g_usNodeId<<"\n";
	// LogInfo(msg.str());

	// error.data=arm_control.angle-angle.data[2];
	// error_inc=error.data*16384*81/360;
    // printf("error= %lf \n",error.data);
    // printf("Array value= %lf \n",angle.data[2]);
	// pub.publish(error);

    // if(angle.data[2]>arm_control.angle-1 && angle.data[2]<arm_control.angle+1)
    // {
	// 	HaltPosition(g_pKeyHandle,g_usNodeId);
    // }
    // MoveToPosition(g_pKeyHandle, g_usNodeId, error_inc,relative, &ulErrorCode);

	// int position_new;
	// get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);
	//HaltPosition(g_pKeyHandle,g_usNodeId);

}



int main(int argc, char **argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	long error_inc;

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

	ros::init(argc, argv, "epos_imu_position");
	ros::NodeHandle n;


	ros::Publisher pub=n.advertise<std_msgs::Float64>("pid_position",1000);
	ros::Rate loop_rate(100);
	ros::Subscriber sub1 = n.subscribe("flexion_angle", 1000, arrayCallback);
	ros::Subscriber sub = n.subscribe("exoskel_control", 1000, moveArm);
	
    
	while(ros::ok())
	{
		// printf("Entered while loop");
		if(arm_control.mode == 1)
		{
			stringstream msg;
			int absolute=1;
			int relative=0;
			float Kp=1;

			msg << "move to position = " << (long)arm_control.angle << ", node = " << g_usNodeId<<"\n";
			LogInfo(msg.str());

			error.data=-(arm_control.angle-angle.data[2]);
			error_inc=Kp*error.data*16384*81/360;
    		printf("error= %lf \n",error.data);

			if(angle.data[2]>arm_control.angle-0.5 && angle.data[2]<arm_control.angle+0.5)
    		{
				HaltPosition(g_pKeyHandle,g_usNodeId);
    		}
			else
			{
				MoveToPosition(g_pKeyHandle, g_usNodeId, error_inc,relative, &ulErrorCode);
			}
		}
		// else
		// {
		// 	SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
		// }
		
	pub.publish(error);
	ros::spinOnce();

	}
	// ros::spin();

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