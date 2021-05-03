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

    // Print maxon headerline
    PrintHeader();

	// Set parameter for usb IO operation
	SetDefaultParameters();

    //Print default parameters
    PrintSettings2();


	// open device
	if((lResult = OpenDevice2(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice2", lResult, ulErrorCode);
		return lResult;
	}
    VCS_ClearFault(g_pKeyHandle2, g_usNodeId2, &ulErrorCode);
    // printf("Fault Cleared");

    // set enable state
	if((lResult = SetEnableState(g_pKeyHandle2,g_usNodeId2,&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("EnableState", lResult, ulErrorCode);
		return lResult;
	}
	
	if((lResult = ActivateProfileVelocityMode(g_pKeyHandle2,g_usNodeId2,&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("Activate Mode", lResult, ulErrorCode);
		return lResult;
	}

    // get_VelocityProfile(g_pKeyHandle,g_usNodeId,&ulErrorCode);
	ros::init(argc, argv, "epos_imu_velocity2");
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
			float Kp=15;
			float Kd=0.0003;
			int current_velocity;
            long desired_velocity;


			msg << "move to position = " << (long)arm_control.angle << ", node = " << g_usNodeId2<<"\n";
			LogInfo(msg.str());

			error.data=(arm_control.angle-angle.data[2]);

			proportional=Kp*error.data;
			derivative=Kd*((error.data-error_prev.data)/0.005);
			error_prev.data=error.data;

			desired_velocity=-(proportional+derivative);

            if(desired_velocity>400)
            {
                desired_velocity=400;
            }
            else if (desired_velocity<-400)
            {
                desired_velocity=-400;
            }

    		printf("error= %lf \n",error.data);

			if(angle.data[2]>arm_control.angle-0.2 && angle.data[2]<arm_control.angle+0.2)
    		{
				HaltVelocity(g_pKeyHandle2,g_usNodeId2);
				
    		}
			else
			{   
				MoveWithVelocity(g_pKeyHandle2, g_usNodeId2,desired_velocity,&ulErrorCode);
				get_velocity(g_pKeyHandle2,g_usNodeId2,&current_velocity,&ulErrorCode);
			}
		}

	pub.publish(error);
	ros::spinOnce();

	}
	// ros::spin();

	//disable epos
	SetDisableState(g_pKeyHandle2, g_usNodeId2, &ulErrorCode);
	//close device
	if((lResult = CloseDevice2(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice2", lResult, ulErrorCode);
		return lResult;
	}
	return 0;

}