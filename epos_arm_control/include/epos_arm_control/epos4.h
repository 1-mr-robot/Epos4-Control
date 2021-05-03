#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

#include "Definitions.h"

using namespace std;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

extern void* g_pKeyHandle;
extern void* g_pKeyHandle2;
extern void* g_pKeyHandle3;
extern unsigned short g_usNodeId;
extern unsigned short g_usNodeId2;
extern unsigned short g_usNodeId3;
extern string g_deviceName;
extern string g_protocolStackName;
extern string g_interfaceName;
extern string g_portName;
extern string g_portName2;
extern string g_portName3;
extern int g_baudrate ;

typedef void* HANDLE;
typedef int BOOL;

void LogInfo(string message);
void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);

//void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
void  PrintSettings2();
void  PrintSettings3();

int OpenDevice(unsigned int* p_pErrorCode);
int OpenDevice2(unsigned int* p_pErrorCode);
int OpenDevice3(unsigned int* p_pErrorCode);
int SetEnableState(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* p_pErrorCode);
int SetDisableState(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* pErrorCode);
int CloseDevice(unsigned int* p_pErrorCode);// if not close epos, it will keep at position or velocity
int CloseDevice2(unsigned int* p_pErrorCode);
int CloseDevice3(unsigned int* p_pErrorCode);

int ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode);
int ActivateProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode);
int ActivateProfileCurrentMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_rlErrorCode);

void SetDefaultParameters();
void get_position(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* pPositionIs, unsigned int* p_pErrorCode);
void get_PositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
void set_PositionProfile(HANDLE p_DeviceHandle,unsigned short p_usNodeId, unsigned int pProfileVelocity,unsigned int pProfileAcceleration,unsigned int pProfileDeceleration, unsigned int* p_pErrorCode);
void get_VelocityProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
void set_VelocityProfile(HANDLE p_DeviceHandle,unsigned short p_usNodeId, unsigned int pProfileAcceleration,unsigned int pProfileDeceleration, unsigned int* p_pErrorCode);
void get_velocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int* pVelocityIs, unsigned int* p_pErrorCode);
void get_TargetVelocity(void* g_pKeyHandle, unsigned short g_usNodeId, unsigned int* ulErrorCode);// only print
void get_current(HANDLE p_DeviceHandle, unsigned short p_usNodeId, short* pCurrentMust, unsigned int* p_pErrorCode);

//can also move under velocity mode, but have speed limit
int MoveToPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long TargetPosition, int Absolute, unsigned int* p_pErrorCode);
int MoveWithVelocity(void* p_DeviceHandle, unsigned short p_usNodeId, long TargetVelocity, unsigned int* p_pErrorCode);
int SetCurrentMust(HANDLE p_DeviceHandle, unsigned short p_usNodeId, short CurrentMust, unsigned int* p_pErrorCode);
bool HaltVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId);
bool HaltPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId);