#include "epos4.h"

void* g_pKeyHandle;
//void* g_pKeyHandle2;
unsigned short g_usNodeId;
//unsigned short g_usNodeId2;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
//string g_portName2;
int g_baudrate;

const string g_programName = "Epos4 Control";

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintHeader()
{
	SeparatorLine();

	LogInfo("Epos4 Exoskeleton Arm Program");

	SeparatorLine();
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
	// exchange the node id if it fails to write command to epos2
	// with two epos2 connected, it always success in connect
	// but if we get the node id wrong, it fails to write command to epos2 

	g_usNodeId = 1;
	//g_usNodeId2 = 2;
	g_deviceName = "EPOS4"; //EPOS version
	g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
	g_interfaceName = "USB"; //RS232
	g_portName = "USB0"; // /dev/ttyS1
	//g_portName2 = "USB1";
	g_baudrate = 1000000; //115200
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int SetEnableState(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	LogInfo("Enable EPOS");
	return lResult;
}

int SetDisableState(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;
	if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
	{
		LogError("VCS_SetDisableState", lResult, lErrorCode);
		lResult = MMC_FAILED;
	}
	LogInfo("Disable EPOS");
	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, *p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	return  lResult;
}

int ActivateProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile velocity mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, *p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	return  lResult;
}

int ActivateProfileCurrentMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile current mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateCurrentMode(p_DeviceHandle, p_usNodeId, p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateCurrentMode", lResult, *p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	return  lResult;
}

void get_position(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* pPositionIs, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int pErrorCode;
	stringstream msg;
	if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, pPositionIs, &pErrorCode) == 0)
	{
		LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "pPositionIs," << *pPositionIs ;
	LogInfo(msg.str());
}
void get_PositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	stringstream msg;
	int lResult = MMC_SUCCESS;
	unsigned int pProfileVelocity, pProfileAcceleration, pProfileDeceleration, pErrorCode;
	if(VCS_GetPositionProfile(p_DeviceHandle, p_usNodeId, &pProfileVelocity, &pProfileAcceleration, &pProfileDeceleration, &pErrorCode) == 0)
	{
		LogError("VCS_GetPositionProfile", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "\n" 
		<< "pProfileVelocity," << pProfileVelocity 
		<< "pProfileAcceleration," << pProfileAcceleration 
		<< "pProfileDeceleration," << pProfileDeceleration 
		<< "pErrorCode" << pErrorCode;
	LogInfo(msg.str());
}

void get_velocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* pVelocityIs, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	// int pVelocityIs;
	stringstream msg;
	if(VCS_GetVelocityIs(p_DeviceHandle, p_usNodeId, pVelocityIs, p_pErrorCode) == 0)
	{
		LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "pVelocityIs," << *pVelocityIs ;
	LogInfo(msg.str());
}

void get_TargetVelocity(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* ulErrorCode)
{
	int lResult = MMC_SUCCESS;
	long pTargetVelocity;
	stringstream msg;
	if(VCS_GetTargetVelocity(g_pKeyHandle, g_usNodeId, &pTargetVelocity, ulErrorCode) == 0)
	{
		LogError("VCS_GetVelocityIs", lResult, *ulErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "pTargetVelocity," << pTargetVelocity ;
	LogInfo(msg.str());
}

void get_current(HANDLE p_DeviceHandle, unsigned short p_usNodeId, short* pCurrentMust, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	if(VCS_GetCurrentMust(p_DeviceHandle, p_usNodeId, pCurrentMust, p_pErrorCode) == 0)
	{
		LogError("VCS_GetCurrentMust", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "pCurrentMust," << *pCurrentMust ;
	LogInfo(msg.str());
}

int MoveToPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long TargetPosition, int Absolute, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, TargetPosition, Absolute, 0, p_pErrorCode) == 0)
	{
		LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	// msg << "moveToPosition:" << TargetPosition ;
	// LogInfo(msg.str());
	return lResult;
}

int MoveWithVelocity(void* p_DeviceHandle, unsigned short p_usNodeId, long TargetVelocity, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, TargetVelocity, p_pErrorCode) == 0)
	{
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "MoveWithVelocity:" << TargetVelocity ;
	LogInfo(msg.str());
	return lResult;
}

int SetCurrentMust(HANDLE p_DeviceHandle, unsigned short p_usNodeId, short CurrentMust, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	if(VCS_SetCurrentMust(p_DeviceHandle, p_usNodeId, CurrentMust, p_pErrorCode) == 0)
	{
		LogError("VCS_SetCurrentMust", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	msg << "SetCurrentMust:" << CurrentMust;
	// LogInfo(msg.str());
	return lResult;
}

// int ModifiedProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode,  long motor_position)
// {
// 	int lResult = MMC_SUCCESS;
// 	stringstream msg;

// 	msg << "set profile position mode, node = " << p_usNodeId;
// 	LogInfo(msg.str());

// 	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
// 	{
// 		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
// 		lResult = MMC_FAILED;
// 	}

// 	else
// 	{
// 		    long targetPosition = motor_position;
// 			stringstream msg;
// 			msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
// 			LogInfo(msg.str());

// 			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 1, 1, &p_rlErrorCode) == 0)
// 			{
// 				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
// 				lResult = MMC_FAILED;
// 			}
// 			sleep(1);

//             if(lResult == MMC_SUCCESS)
// 		    {
// 			    LogInfo("halt position movement");

// 			    if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
// 			    {
// 				    LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
// 				    lResult = MMC_FAILED;
// 			    }
// 		    }
// 	    }

// 	return lResult;
// }

// bool ModifiedProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long motor_speed)
// {
// 	int lResult = MMC_SUCCESS;
// 	stringstream msg;

// 	msg << "set profile velocity mode, node = " << p_usNodeId;

// 	LogInfo(msg.str());

// 	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
// 	{
// 		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
// 		lResult = MMC_FAILED;
// 	}

// 	else
// 	{
// 			long targetvelocity = motor_speed;
// 			stringstream msg;
// 			msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
// 			LogInfo(msg.str());

// 			if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
// 			{
// 				lResult = MMC_FAILED;
// 				LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
// 			}
// 	    }

// 	return lResult;
// }

// int ModifiedProfileCurrentMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode,short targetCurrent)
// {
// 	int lResult = MMC_SUCCESS;
// 	stringstream msg;

// 	msg << "set profile current mode, node = " << p_usNodeId;
// 	LogInfo(msg.str());

// 	if(VCS_ActivateCurrentMode(p_DeviceHandle, p_usNodeId, p_rlErrorCode) == 0)
// 	{
// 		LogError("VCS_ActivateCurrentMode", lResult, *p_rlErrorCode);
// 		lResult = MMC_FAILED;
// 	}

// 	else
// 	{
// 			// short targetCurrent=280;
// 			stringstream msg;
// 			if(VCS_SetCurrentMust(p_DeviceHandle, p_usNodeId, targetCurrent, p_rlErrorCode) == 0)
// 				{
// 					LogError("VCS_SetCurrentMust", lResult, *p_rlErrorCode);
// 					lResult = MMC_FAILED;
// 				}
// 			msg << "SetCurrentMust (in mA):" << targetCurrent;
// 			LogInfo(msg.str());
// 			// sleep(2);
// 	}

// 	return lResult;
// }


bool HaltVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId)
{
	int lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode=0;
	LogInfo("halt velocity movement");

			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
			return lResult;
}

bool HaltPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId)
{
    int lResult=MMC_SUCCESS;
    unsigned int p_rlErrorCode=0;
	LogInfo("halt position movement");

			    if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			    {
				    LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				    lResult = MMC_FAILED;
			    }
                return lResult;
}

	