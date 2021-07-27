#pragma once
///////////////////////////////////////////////////////////////////////////////
// FILE:          NnCameraDemo.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// AUTHOR:        Zhang Ren, zhangren@tucsen.com, 27/03/2017
//
// COPYRIGHT:     Tucsen Photonics Co., Ltd.  2018
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

#ifndef _NNCAMERADEMO_H_
#define _NNCAMERADEMO_H_

#include "nncam.h"
#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_UNKNOWN_MODE         102
#define ERR_UNKNOWN_POSITION     103
#define ERR_IN_SEQUENCE          104
#define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         106
#define HUB_NOT_AVAILABLE        107

const char* NoHubError = "Parent Hub not defined.";


// Defines which segments in a seven-segment display are lit up for each of
// the numbers 0-9. Segments are:
//
//  0       1
// 1 2     2 4
//  3       8
// 4 5    16 32
//  6      64
const int SEVEN_SEGMENT_RULES[] = {1+2+4+16+32+64, 4+32, 1+4+8+16+64,
      1+4+8+32+64, 2+4+8+32, 1+2+8+32+64, 2+8+16+32+64, 1+4+32,
      1+2+4+8+16+32+64, 1+2+4+8+32+64};
// Indicates if the segment is horizontal or vertical.
const int SEVEN_SEGMENT_HORIZONTALITY[] = {1, 0, 0, 1, 0, 0, 1};
// X offset for this segment.
const int SEVEN_SEGMENT_X_OFFSET[] = {0, 0, 1, 0, 0, 1, 0};
// Y offset for this segment.
const int SEVEN_SEGMENT_Y_OFFSET[] = {0, 0, 0, 1, 1, 1, 2};

////////////////////////
// DemoHub
//////////////////////
#define TUDBG_PRINTF(format,...)	{char dbgMsg[1024] = {0}; sprintf_s(dbgMsg, "" format"", ##__VA_ARGS__); OutputDebugString(dbgMsg);}
///////////////////////
// CameraPID
///////////////////////
#define DHYANA_400D_X45     0x6404
#define DHYANA_D95_X45      0x6405
#define DHYANA_400DC_X45    0x6804
#define DHYANA_400DC_X100   0xEC03
#define DHYANA_400D_X100    0xE404
#define DHYANA_400BSIV1     0xE405
#define DHYANA_D95_X100     0xE406
#define DHYANA_400BSIV2     0xE408
#define PID_FL_20BW         0xE40D
#define DHYANA_D95_V2       0xE40F
#define DHYANA_401D         0xE005

///////////////////////
// ImgMode
///////////////////////
#define MODE_12BIT    0x00
#define MODE_CMS      0x01
#define MODE_11BIT    0x02
#define MODE_GLRESET  0x03

class DemoHub : public HubBase<DemoHub>
{
public:
   DemoHub() :
      initialized_(false),
      busy_(false)
   {}
   ~DemoHub() {}

   // Device API
   // ---------
   int Initialize();
   int Shutdown() {return DEVICE_OK;};
   void GetName(char* pName) const; 
   bool Busy() { return busy_;} ;

   // HUB api
   int DetectInstalledDevices();

private:
   void GetPeripheralInventory();

   std::vector<std::string> peripherals_;
   bool initialized_;
   bool busy_;
};


//////////////////////////////////////////////////////////////////////////////
// CNnCameraDemo class
// Simulation of the Camera device
//////////////////////////////////////////////////////////////////////////////

class CNNCamDemoThread;

class CNnCameraDemo : public CCameraBase<CNnCameraDemo>  
{
public:
    CNnCameraDemo();
    ~CNnCameraDemo();
  
    // MMDevice API
    // ------------
    int Initialize();
    int Shutdown();

    void GetName(char* name) const;      

    // MMCamera API
    // ------------
    int SnapImage();
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

    int PrepareSequenceAcqusition() { return DEVICE_OK; }
    int StartSequenceAcquisition(double interval);
    int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
    int StopSequenceAcquisition();
    int InsertImage();
    int RunSequenceOnThread(MM::MMTime startTime);
    bool IsCapturing();
    void OnThreadExiting() throw(); 
    double GetNominalPixelSizeUm() const {return nominalPixelSizeUm_;}
    double GetPixelSizeUm() const {return nominalPixelSizeUm_ * GetBinning();}
    int GetBinning() const;
    int SetBinning(int bS);

    int IsExposureSequenceable(bool& isSequenceable) const;
    int GetExposureSequenceMaxLength(long& nrEvents) const;
    int StartExposureSequence();
    int StopExposureSequence();
    int ClearExposureSequence();
    int AddToExposureSequence(double exposureTime_ms);
    int SendExposureSequence() const;

    unsigned  GetNumberOfComponents() const { return nComponents_;};

    // action interface
    // ----------------
    // floating point read-only properties for testing
    int OnTestProperty(MM::PropertyBase* pProp, MM::ActionType eAct, long);

    int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPixelClock(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnExposure(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnGlobalGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnCMSMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnLEDMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnGAINMode(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnImageMode(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFlipH(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFlipV(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnGamma(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnContrast(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnSaturation(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnWhiteBalance(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnRedGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnGreenGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnBlueGain(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnATExposure(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnTemperature(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFan(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnLeftLevels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnRightLevels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnImageFormat(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTriggerMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTriggerExpMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTriggerEdgeMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTriggerDelay(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTriggerDoSoftware(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSharpness(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnDPCLevel(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDPCAdjust(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnBlackLevel(MM::PropertyBase* pProp, MM::ActionType eAct);

	int OnTrgOutPortMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTrgOutKindMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTrgOutEdgeMode(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTrgOutDelay(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnTrgOutWidth(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnErrorSimulation(MM::PropertyBase* , MM::ActionType );
    int OnCameraCCDXSize(MM::PropertyBase* , MM::ActionType );
    int OnCameraCCDYSize(MM::PropertyBase* , MM::ActionType );
    int OnTriggerDevice(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnCCDTemp(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnIsSequenceable(MM::PropertyBase* pProp, MM::ActionType eAct);


private:
    int SetAllowedBinning();
    int SetAllowedPixelClock();
    int SetAllowedFanGear();
    int SetAllowedImageMode();

    void TestResourceLocking(const bool);
    void GenerateEmptyImage(ImgBuffer& img);
    void GenerateSyntheticImage(ImgBuffer& img, double exp);
    int ResizeImageBuffer();

    static const double nominalPixelSizeUm_;

    double exposureMaximum_;
    double exposureMinimum_;
    double dPhase_;
    ImgBuffer img_;
    bool busy_;
    bool stopOnOverFlow_;
    bool initialized_;
    double readoutUs_;
    MM::MMTime readoutStartTime_;
    long scanMode_;
    int bitDepth_;
    unsigned roiX_;
    unsigned roiY_;
    MM::MMTime sequenceStartTime_;
    bool isSequenceable_;
    long sequenceMaxLength_;
    bool sequenceRunning_;
    unsigned long sequenceIndex_;
    double GetSequenceExposure();
    std::vector<double> exposureSequence_;
    long imageCounter_;
    long binSize_;
    long cameraCCDXSize_;
    long cameraCCDYSize_;
    double ccdT_;
    std::string triggerDevice_;

    bool stopOnOverflow_;

    bool dropPixels_;
    bool fastImage_;
	//bool snapFlag_ ;
    bool saturatePixels_;
    double fractionOfPixelsToDropOrSaturate_;
    bool shouldRotateImages_;
    bool shouldDisplayImageNumber_;
    double stripeWidth_;

    double testProperty_[10];
    MMThreadLock imgPixelsLock_;
    int nComponents_;
    bool returnToSoftwareTriggers_;

    friend class CNNCamDemoThread;
    CNNCamDemoThread * thd_;

	//HNncam g_hcam ;


private:
	void TestImage(ImgBuffer& img, double exp);

    static void __cdecl GetTemperatureThread(LPVOID lParam);    // The thread get the value of temperature
    
	void RunTemperature();

    int InitNNCamDemoApi();
    int UninitNNCamDemoApi();

    int AllocBuffer();
    int ResizeBuffer();
    int ReleaseBuffer();

    int StopCapture();
    int StartCapture();
	int RestartCapture();
    int WaitForFrame(ImgBuffer& img);
	int CopyToFrame(ImgBuffer& img,int nWidth,int nHeight,int CopyToFrame);


    bool SaveRaw(char *pfileName, unsigned char *pData, unsigned long ulSize);

	//void __stdcall EventCallback(unsigned nEvent, void* pCallbackCtx);

	static int   	s_nNumCam;				// The number of cameras
	static int		s_nCntCam;				// The count of camera

	int             m_nPID;                 // The PID 
	int             m_nBCD;                 // The BCD
    int             m_nIdxGain;             // The gain mode
    int             m_nMaxHeight;           // The max height size
    char            m_szImgPath[MAX_PATH];  // The save image path
    float           m_fCurTemp;             // The current temperature
    float           m_fValTemp;             // The temperature value
    int             m_nMidTemp;             // The middle value of temperature
    bool            m_bROI;                 // The ROI state
    bool            m_bSaving;              // The tag of save image            
    bool            m_bTemping;             // The get temperature state
    bool            m_bLiving;              // The capturing state
	HANDLE          m_hThdWaitEvt;          // The waiting frame thread event handle
    HANDLE          m_hThdTempEvt;          // To get the value of temperature event handle

	float           maxGain;                //最大增益值
	float           minGain;                //最小增益值

	int             binVal ;


	//static HNncam g_hcam ;
	//void* g_pImageData;
	//unsigned g_total;

	BITMAPINFOHEADER	m_header;
	/*
    TUCAM_INIT       m_itApi;               // The initialize SDK environment
    TUCAM_OPEN       m_opCam;               // The camera open parameters
    TUCAM_FRAME      m_frame;               // The frame object
	TUCAM_TRIGGER_ATTR m_tgrAttr;			// The trigger parameters
	TUCAM_TRGOUT_ATTR  m_tgrOutAttr;        // The output trigger parameters
	TUCAM_TRGOUTPUT    m_tgrOutPara;        // The output trigger parameter port
	*/
	//unsigned short   mbiBitCount ;
};


class CNNCamDemoThread : public MMDeviceThreadBase
{
   friend class CNnCameraDemo;
   enum { default_numImages=1, default_intervalMS = 100 };
   public:
      CNNCamDemoThread(CNnCameraDemo* pCam);
      ~CNNCamDemoThread();
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
      double intervalMs_;                                                       
      long numImages_;                                                          
      long imageCounter_;                                                       
      bool stop_;                                                               
      bool suspend_;                                                            
      CNnCameraDemo* camera_;                                                     
      MM::MMTime startTime_;                                                    
      MM::MMTime actualDuration_;                                               
      MM::MMTime lastFrameTime_;                                                
      MMThreadLock stopLock_;                                                   
      MMThreadLock suspendLock_;                                                
}; 





#endif //_NNCAMERADEMO_H_



