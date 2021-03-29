#include "ros/ros.h"
#include "epos4.h"
#include <epos_arm_control/epos.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

epos_arm_control::epos arm_control;
std_msgs::Float64MultiArray angle;
std_msgs::Float64 error;
std_msgs::Float64 error_prev;



void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{

	angle=*array;

}


void moveArm(const epos_arm_control::epos::ConstPtr& params)
{

	arm_control=*params;

}



int main(int argc, char **argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	long error_inc;
	float proportional;
	float derivative;
	unsigned int ProfileVelocity=100, ProfileAcceleration=100, ProfileDeceleration=100;

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

	set_PositionProfile(g_pKeyHandle,g_usNodeId,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&ulErrorCode);

	ros::init(argc, argv, "epos_imu_position");
	ros::NodeHandle n;


	ros::Publisher pub=n.advertise<std_msgs::Float64>("pid_position",1000);
	ros::Rate loop_rate(200);
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
			float Kd=0.0007;
			float desired_angle=0;
			long desired_inc=0;


			msg << "move to position = " << (long)arm_control.angle << ", node = " << g_usNodeId<<"\n";
			LogInfo(msg.str());

			error.data=(arm_control.angle-angle.data[2]);

			proportional=Kp*error.data;
			derivative=Kd*((error.data-error_prev.data)/0.005);
			error_prev.data=error.data;

			desired_angle=proportional+derivative;
			desired_inc=desired_angle*16384*81/360;

    		printf("error= %lf \n",error.data);

			if(angle.data[2]>arm_control.angle-0.2 && angle.data[2]<arm_control.angle+0.2)
    		{
				//HaltVelocity(g_pKeyHandle,g_usNodeId);
				HaltPosition(g_pKeyHandle,g_usNodeId);
				
    		}
			else
			{
				MoveToPosition(g_pKeyHandle, g_usNodeId, desired_inc,relative, &ulErrorCode);
				get_PositionProfile(g_pKeyHandle,g_usNodeId,&ulErrorCode);
			}
		}
		
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