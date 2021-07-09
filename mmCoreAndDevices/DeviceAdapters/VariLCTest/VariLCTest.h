#pragma once


#ifndef _VARILCTEST_H_
#define _VARILCTEST_H_


#include "MMDevice.h"
#include "DeviceBase.h"


// MMCore name of serial port
std::string port_;

int ClearPort(MM::Device& device, MM::Core& core, const char* port);

class VariLCTest : public CGenericBase<VariLCTest>
{
   public:
	VariLCTest();
	~VariLCTest();


	// Device API
    // ---------
	int Initialize();
	int Shutdown();

	void GetName(char* pszName) const;
	bool Busy();
	int OnDelay (MM::PropertyBase* pProp, MM::ActionType eAct);


    // device discovery
    bool SupportsDeviceDetection(void);
	MM::DeviceDetectionStatus DetectDevice(void);

	 // action interface
    // ---------------
	int OnPort    (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnWavelength (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnBriefMode (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSerialNumber (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnNumTotalLCs (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnNumActiveLCs (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnRetardance (MM::PropertyBase* pProp, MM::ActionType eAct, long index);
	int OnAbsRetardance (MM::PropertyBase* pProp, MM::ActionType eAct, long index);
	int OnPalEl (MM::PropertyBase* pProp, MM::ActionType eAct, long index);
	int OnSendToVariLC (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnGetFromVariLC (MM::PropertyBase* pProp, MM::ActionType eAct);


   private:

	 std::string baud_;
	 bool briefModeQ_;
	 double wavelength_; // the cached value
	 bool initialized_;
	 bool initializedDelay_;
	 long numTotalLCs_;  // total number of LCs
	 long numActiveLCs_;  // number of actively controlled LCs (the actively controlled LCs appear first in the list of retardance values in the L-command)
	 double retardance_[25]; // retardance values of total number of LCs; I made the index 8, a high number unlikely to be exceeded by the variLC hardware
	 long numPalEls_;  // total number of palette elements
	 std::string palEl_[99]; // array of palette elements, index is total number of elements
	 std::string serialnum_;
	 std::string sendToVariLC_;
	 
	 std::string getFromVariLC_;
     MM::MMTime changedTime_;
	 MM::MMTime delay;
	 std::vector<double> getNumbersFromMessage(std::string variLCmessage, bool prefixQ);


	 std::string DoubleToString(double N);
	 int sendCmd(std::string cmd, std::string& out);	//Send a command and save the response in `out`.
	 int sendCmd(std::string cmd);	//Send a command that does not repond with any extra information.

};


#endif //_VARILCTEST_H_
