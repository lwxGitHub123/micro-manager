#pragma once
///////////////////////////////////////////////////////////////////////////////
// FILE:          CNnCamera2.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Implements the Raptor camera device adaptor using EPIX frame grabber.
//				  Supported cameras: Falcon, Kite and OWL.
//				  DemoCamera.cpp modified by KBIS for Raptor Photonics camera support
//                
// AUTHOR:        DB @ KBIS, 10/15/2012
//
// COPYRIGHT:     Raptor Photonics Ltd, (2011, 2012)
// LICENSE:       License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES. 
//

#ifndef _NNCAMERA2_H_
#define _NNCAMERA2_H_


//#include "MMDevice.h"
#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include "DeviceUtils.h"
#include <string>
#include <map>




//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_UNKNOWN_MODE         102

#define ERR_CAMERA_NOT_FOUND     103
#define ERR_MEM_ALLOC            105
#define ERR_ROI_INVALID          110


// MMCore name of serial port
std::string port_;

//int ClearPort(MM::Device& device, MM::Core& core, const char* port);
int ClearPort(MM::Device& device, MM::Core& core, std::string port);


class MySequenceThread;

class CNnCamera2: public CCameraBase<CNnCamera2>
{
public:
	CNnCamera2();
	~CNnCamera2();


	char* pcImgMem;                                       //image memory
    int memPid;                                           //ID for image memory


	// MMDevice API
    int Initialize();
    int Shutdown();
	int SnapImage( void );
	const unsigned char* GetImageBuffer();
	unsigned GetImageWidth() const;
	unsigned GetImageHeight() const;
	unsigned GetImageBytesPerPixel() const;
	unsigned GetBitDepth() const;
	long GetImageBufferSize() const;
    double GetExposure() const;
	void SetExposure(double exp);
	int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize); 
	int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize); 
	int ClearROI();
	int StartSequenceAcquisition(double interval);
    int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
	int StopSequenceAcquisition();


	int GetBinning() const;
	int SetBinning(int bS);


	void GetName(char* pszName) const;
	
	


	 // action interface
     // ---------------	
	int OnGetFromVariLC (MM::PropertyBase* pProp, MM::ActionType eAct);
	int IsExposureSequenceable(bool& isSequenceable) const {isSequenceable = false; return DEVICE_OK;}
	//int ClearPort(MM::Device& device, MM::Core& core, std::string port);
	int OnPort    (MM::PropertyBase* pProp, MM::ActionType eAct);

	//int OnCameraType(MM::PropertyBase* pProp, MM::ActionType eAct);


	// device discovery
	bool SupportsDeviceDetection(void);
	MM::DeviceDetectionStatus DetectDevice(void);

	int OnDemoMode(MM::PropertyBase* pProp, MM::ActionType eAct);

	void WriteLog(char *message, int err);


private:

	ImgBuffer img_;

	std::string baud_;

	long numTotalNNc_;  // total number of nnCamera
	double retardance_[25]; // retardance values of total number of LCs; I made the index 8, a high number unlikely to be exceeded by the variLC hardware
	long numPalEls_;  // total number of palette elements
	std::string palEl_[99]; // array of palette elements, index is total number of elements

	bool initializedDelay_;
	bool briefModeQ_;
	long numTotalLCs_;  // total number of LCs	 
	long numActiveLCs_;  // number of actively controlled LCs (the actively controlled LCs appear first in the list of retardance values in the L-command)
	float pictime_;
	double wavelength_; // the cached value

	MMThreadLock imgPixelsLock_;
	double readoutUs_;
	bool stopOnOverflow_;
	MM::MMTime readoutStartTime_;
	int bitDepth_;

	unsigned roiX_;                                               //x-position of the ROI corner
    unsigned roiY_;                                               //y-position of the ROI corner
    unsigned roiXSize_;                                           //width of the ROI, screen pixel (incl. binning)
    unsigned roiYSize_;                                           //height of the ROI, screen pixel (incl. binning)
	unsigned roiXSizeReal_;                                       //width of the ROI, CCD pixel
    unsigned roiYSizeReal_;                                       //height of the ROI, CCD pixel

	long cameraCCDXSize_;                                         //sensor width in pixels
    long cameraCCDYSize_;                                         //sensor height in pixels

	MM::MMTime sequenceStartTime_;
	long imageCounter_;

	double exposureSet_;                                          //set (desired) exposure value

	long binX_;                                                   //horizontal binning factor
    long binY_;                                                   //vertical binning factor


	std::string getFromVariLC_;

	int m_iCameraNum;
	std::string serialnum_;

	//std::vector<double> getNumbersFromMessage(std::string variLCmessage, bool prefixQ);

	bool initialized_;


	std::string sendToVariLC_;
	MM::MMTime changedTime_;
	MM::MMTime delay;
	std::string DoubleToString(double N);
	int sendCmd(std::string cmd, std::string& out);	//Send a command and save the response in `out`.
	int sendCmd(std::string cmd);	//Send a command that does not repond with any extra information.

    void ClearImageBuffer(ImgBuffer& img);
    int SetImageMemory();

	friend class MySequenceThread;
    MySequenceThread * thd_;

	int UNITSOPENMAP;
	int MULTIUNITMASK;


};



class MySequenceThread : public MMDeviceThreadBase
{

	  friend class CNnCamera2;
	  enum { default_numImages=1, default_intervalMS = 100 };

	 public:
	  MySequenceThread(CNnCamera2* pCam);
	  ~MySequenceThread();

	  void Stop();
	  void Start(long numImages, double intervalMs);
	  bool IsStopped();
	  void Suspend();
	  bool IsSuspended();
	  void Resume();
	  double GetIntervalMs(){return intervalMs_;}                               
	  void SetLength(long images) {numImages_ = images;}                        
	  long GetLength() const {return numImages_;}
	  long GetImageCounter(){return imageCounter_;}                             
	  MM::MMTime GetStartTime(){return startTime_;}                             
	  MM::MMTime GetActualDuration(){return actualDuration_;}

	 private:                                                                     
	  int svc(void) throw();
	  CNnCamera2* camera_;                                                     
	  bool stop_;                                                               
	  bool suspend_;                                                            
	  long numImages_;                                                          
	  long imageCounter_;                                                       
	  double intervalMs_;                                                       
	  MM::MMTime startTime_;                                                    
	  MM::MMTime actualDuration_;                                               
	  MM::MMTime lastFrameTime_;                                                
	  MMThreadLock stopLock_;                                                   
	  MMThreadLock suspendLock_;
}; 

bool m_bDemoMode;




#endif //_NNCAMERA1_H_
