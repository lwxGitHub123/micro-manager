///////////////////////////////////////////////////////////////////////////////
// FILE:          Zaber.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Zaber Controller Driver
//                
// AUTHOR:        David Goosen & Athabasca Witschi (contact@zaber.com)
//                
// COPYRIGHT:     Zaber Technologies Inc., 2014
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

#ifndef _ZABER_H_
#define _ZABER_H_

#include <MMDevice.h>
#include <DeviceBase.h>
#include <ModuleInterface.h>
#include <sstream>
#include <string>

//////////////////////////////////////////////////////////////////////////////
// Various constants: error codes, error messages
//////////////////////////////////////////////////////////////////////////////

#define ERR_PORT_CHANGE_FORBIDDEN    10002
#define ERR_DRIVER_DISABLED          10004
#define ERR_BUSY_TIMEOUT             10008
#define ERR_AXIS_COUNT               10016
#define ERR_COMMAND_REJECTED         10032
#define	ERR_NO_REFERENCE_POS         10064
#define	ERR_SETTING_FAILED           10128
#define	ERR_INVALID_DEVICE_NUM       10256
#define ERR_LAMP_DISCONNECTED        10512
#define ERR_LAMP_OVERHEATED          11024
#define ERR_PERIPHERAL_DISCONNECTED  12048
#define ERR_PERIPHERAL_UNSUPPORTED   14096
#define ERR_FIRMWARE_UNSUPPORTED     18192

extern const char* g_Msg_PORT_CHANGE_FORBIDDEN;
extern const char* g_Msg_DRIVER_DISABLED;
extern const char* g_Msg_BUSY_TIMEOUT;
extern const char* g_Msg_AXIS_COUNT;
extern const char* g_Msg_COMMAND_REJECTED;
extern const char* g_Msg_NO_REFERENCE_POS;
extern const char* g_Msg_SETTING_FAILED;
extern const char* g_Msg_INVALID_DEVICE_NUM;
extern const char* g_Msg_LAMP_DISCONNECTED;
extern const char* g_Msg_LAMP_OVERHEATED;
extern const char* g_Msg_PERIPHERAL_DISCONNECTED;
extern const char* g_Msg_PERIPHERAL_UNSUPPORTED;
extern const char* g_Msg_FIRMWARE_UNSUPPORTED;

// N.B. Concrete device classes deriving ZaberBase must set core_ in
// Initialize().
class ZaberBase
{
public:
	ZaberBase(MM::Device *device);
	virtual ~ZaberBase();

protected:
	int ClearPort() const;
	int SendCommand(const std::string command) const;
	int SendCommand(long device, long axis, const std::string command) const;
	int QueryCommand(const std::string command, std::vector<std::string>& reply) const;
	int QueryCommandUnchecked(const std::string command, std::vector<std::string>& reply) const;
	int QueryCommandUnchecked(long device, long axis, const std::string command, std::vector<std::string>& reply) const;
	int CheckReplyFlags(const std::string reply) const;
	int GetSetting(long device, long axis, std::string setting, long& data) const;
	int GetSetting(long device, long axis, std::string setting, double& data) const;
	int SetSetting(long device, long axis, std::string setting, long data) const;
	int SetSetting(long device, long axis, std::string setting, double data, int decimalPlaces) const;
	bool IsBusy(long device) const;
	int Stop(long device, long lockstepGroup = 0) const;
	int GetLimits(long device, long axis, long& min, long& max) const;
	int SendMoveCommand(long device, long axis, std::string type, long data, bool lockstep = false) const;
	int SendAndPollUntilIdle(long device, long axis, std::string command, int timeoutMs) const;
	int GetRotaryIndexedDeviceInfo(long device, long axis, long& numIndices, long& currentIndex) const;
	int GetFirmwareVersion(long device, double& version) const;
	int ActivatePeripheralsIfNeeded(long device) const;

	bool initialized_;
	std::string port_;
	MM::Device *device_;
	MM::Core *core_;
	std::string cmdPrefix_;
};

#endif //_ZABER_H_
