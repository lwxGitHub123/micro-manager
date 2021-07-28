///////////////////////////////////////////////////////////////////////////////
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// AUTHOR:        Zhang Ren, zhangren@tucsen.com, 27/03/2017
//
// COPYRIGHT:     Tucsen Photonics Co., Ltd., 2018
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

#include "NnCameraDemo.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include "WriteCompactTiffRGB.h"
#include <iostream>
#include <process.h>
#include "Utils.h"
//#include "nncam.h"

#include <opencv2\opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <Windows.h>

#include "ImgProc.h"

/*
#include <shlwapi.h>
#pragma comment(lib, "shlwapi.lib")
*/

using namespace std;
const double CNnCameraDemo::nominalPixelSizeUm_ = 1.0;
double g_IntensityFactor_ = 1.0;

// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
const char* g_NNDemoDeviceName  = "NNCamDemo";
const char* g_PropNameFan   = "Fan";
const char* g_PropNamePCLK  = "PixelClock";
const char* g_PropNameBODP  = "BitDepth";
const char* g_PropNameGain  = "Gain";
const char* g_PropNameMode  = "Gain";
const char* g_PropNameFLPH  = "FlipH";
const char* g_PropNameFLPV  = "FlipV";
const char* g_PropNameGAMM  = "Image Adjustment Gamma";
const char* g_PropNameCONT  = "Image Adjustment Contrast";
const char* g_PropNameSATU  = "Image Adjustment Saturation";
const char* g_PropNameRGAN  = "Image Adjustment Channel R";
const char* g_PropNameGGAN  = "Image Adjustment Channel G";
const char* g_PropNameBGAN  = "Image Adjustment Channel B";
const char* g_PropNameATWB  = "Image Adjustment Auto White Balance";
const char* g_PropNameONWB  = "Image Adjustment Once White Balance";
const char* g_PropNameATEXP = "Exposure_Auto Adjustment";
const char* g_PropNameTEMP  = "Temperature";
const char* g_PropNameLLev  = "Image Adjustment Left  Levels";
const char* g_PropNameRLev  = "Image Adjustment Right Levels";
const char* g_PropNameIFMT  = "SaveImage";
const char* g_PropNameCMS   = "CMSMode";
const char* g_PropNameLED   = "LEDMode";
const char* g_PropNameDepth = "DepthMode";
const char* g_PropNameMdTgr = "Trigger Mode";
const char* g_PropNameMdExp = "Trigger Exposure Mode";
const char* g_PropNameMdEdg = "Trigger Edge Mode";
const char* g_PropNameMdDly = "Trigger Delay";
const char* g_PropNameDoSFW = "Trigger Software Do";
const char* g_PropNameSharp = "Image Adjustment Sharpness";
const char* g_PropNameDPC   = "Image Adjustment DPC";
const char* g_PropNameOffset= "Image Adjustment Offset";
const char* g_PropNamePort  = "Output Trigger Port";
const char* g_PropNameKind  = "Output Trigger Kind";
const char* g_PropNameEdge  = "Output Trigger Edge Mode";
const char* g_PropNameDelay = "Output Trigger Delay";
const char* g_PropNameWidth = "Output Trigger Width";

const char* g_DeviceName = "Vihent";   //"400Li"
const char* g_SdkName = "NNCam";
//const char* g_SdkName = "CamCore";

//const char* g_DeviceName = "Dhyana400A";
//const char* g_SdkName = "CamKernel";

const char* g_Color = "Color Mode";
const char* g_Gray  = "Gray Mode";

const char* g_WB  = "Click WhiteBalance";

const char* g_AE_ON  = "On";
const char* g_AE_OFF = "Off";

const char* g_CMS_ON  = "On";
const char* g_CMS_OFF = "Off";

const char* g_LED_ON  = "On";
const char* g_LED_OFF = "Off";

const char* g_CMSBIT_ON  = "CMS";
const char* g_HDRBIT_ON  = "HDR";
const char* g_HIGHBIT_ON = "HIGH";
const char* g_LOWBIT_ON  = "LOW";
const char* g_GRHIGH_ON  = "GLOBALRESETHIGH";
const char* g_GRLOW_ON   = "GLOBALRESETLOW";
const char* g_STDHIGH_ON = "STDHIGH";
const char* g_STDLOW_ON  = "STDLOW";

const char* g_TRIGGER_OFF = "Off";
const char* g_TRIGGER_STD = "Standard";
const char* g_TRIGGER_SYN = "Synchronization";
const char* g_TRIGGER_GLB = "Global";
const char* g_TRIGGER_SWF = "Software";

const char* g_TRIGGER_PORT1 = "1";
const char* g_TRIGGER_PORT2 = "2";
const char* g_TRIGGER_PORT3 = "3";

const char* g_TRIGGER_EXPSTART = "Exposure Start";
const char* g_TRIGGER_READEND  = "Readout End";
const char* g_TRIGGER_GLBEXP   = "Global Exposure";
const char* g_TRIGGER_LOW      = "Low";
const char* g_TRIGGER_HIGH     = "High";

const char* g_TRIGGER_EXP_EXPTM    = "Timed";
const char* g_TRIGGER_EXP_WIDTH    = "Width";
const char* g_TRIGGER_EDGE_RISING  = "Rising";
const char* g_TRIGGER_EDGE_FALLING = "Falling";

const char* g_TRIGGER_DO_SOFTWARE  = "Exec";

const char* g_DPC_OFF    = "Off";
const char* g_DPC_LOW    = "Low";
const char* g_DPC_MEDIUM = "Medium";
const char* g_DPC_HIGH   = "High";

const char* g_Format_PNG = "PNG";
const char* g_Format_TIF = "TIF";
const char* g_Format_JPG = "JPG";
const char* g_Format_BMP = "BMP";
const char* g_Format_RAW = "RAW";
const char* g_FileName   = "\\Image";

// constants for naming pixel types (allowed values of the "PixelType" property)
const char* g_PixelType_8bit     = "8bit";
const char* g_PixelType_16bit    = "16bit";
const char* g_PixelType_24bitRGB = "24bitRGB";
const char* g_PixelType_32bitRGB = "32bitRGB";
const char* g_PixelType_64bitRGB = "64bitRGB";
const char* g_PixelType_32bit    = "32bit";  // floating point greyscale


#define MSG_CAMEVENT			(WM_APP + 1)


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
   string log = " CNnCameraDemo  NNCSENDemo Camera";
   CUtils::cameraLog(log);
   RegisterDevice(g_NNDemoDeviceName, MM::CameraDevice, "NNCSENDemo Camera");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   string log = " CNnCameraDemo  CreateDevice";
   CUtils::cameraLog(log);

   // decide which device class to create based on the deviceName parameter
   if (strcmp(deviceName, g_NNDemoDeviceName) == 0)
   {
      // create camera
      return new CNnCameraDemo();
   }

   // ...supplied name not recognized
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   string log = " CNnCameraDemo  DeleteDevice";
   CUtils::cameraLog(log);
   delete pDevice;
}



int CNnCameraDemo::s_nNumCam  = 0;
int CNnCameraDemo::s_nCntCam  = 0;
//HNncam g_hcamT ;
HNncam g_hcam ;

/*
void* g_pImageDataT;
unsigned g_totalT;
*/

void* g_pImageData;
unsigned g_total;
unsigned snapCount;


bool m_bPaused = false;
//snapFlag_ 取值为0表示未进入、1表示进入snap、2表示snap刚结束、3表示进入live、4表示离开live
unsigned snapFlag_ ;
unsigned mbiBitCount ;
int nBandNum ;
int nBPB ;

///////////////////////////////////////////////////////////////////////////////
// CNnCameraDemo implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
* CNnCameraDemo constructor.
* Setup default all variables and create device properties required to exist
* before intialization. In this case, no such properties were required. All
* properties will be created in the Initialize() method.
*
* As a general guideline Micro-Manager devices do not access hardware in the
* the constructor. We should do as little as possible in the constructor and
* perform most of the initialization in the Initialize() method.
*/
CNnCameraDemo::CNnCameraDemo():
    CCameraBase<CNnCameraDemo> (),
	exposureMaximum_(2000000.0), 
    exposureMinimum_(244),
	maxGain(5000.0),
	minGain(100.0),
	binVal(1),
    dPhase_(0),
    initialized_(false),
    readoutUs_(0.0),
    scanMode_(1),
    bitDepth_(8),
    roiX_(0),
    roiY_(0),
    sequenceStartTime_(0),
    isSequenceable_(false),
    sequenceMaxLength_(100),
    sequenceRunning_(false),
    sequenceIndex_(0),
    binSize_(1),
    cameraCCDXSize_(512),
    cameraCCDYSize_(512),
    ccdT_ (0.0),
    triggerDevice_(""),
    stopOnOverflow_(false),
    dropPixels_(false),
    fastImage_(false),
    saturatePixels_(false),
    fractionOfPixelsToDropOrSaturate_(0.002),
    shouldRotateImages_(false),
    shouldDisplayImageNumber_(false),
    stripeWidth_(1.0),
    nComponents_(1),
    returnToSoftwareTriggers_(false)
	//g_hcam(NULL),
	//g_pImageData(NULL),
	//g_total(0),
	//mbiBitCount(24)
{
    memset(testProperty_,0,sizeof(testProperty_));

    // call the base class method to set-up default error codes/messages
    InitializeDefaultErrorMessages();
    readoutStartTime_ = GetCurrentMMTime();
    thd_ = new CNNCamDemoThread(this);

	string log = " CNnCameraDemo  CNnCameraDemo";
    CUtils::cameraLog(log);


}


CNnCameraDemo::~CNnCameraDemo()
{

    string log = " CNnCameraDemo  ~CNnCameraDemo";
    CUtils::cameraLog(log);
    if (m_hThdTempEvt != NULL)
	{
        m_bTemping = false;
		WaitForSingleObject(m_hThdTempEvt, INFINITE);	
		CloseHandle(m_hThdTempEvt);
		m_hThdTempEvt = NULL;
	}

    StopSequenceAcquisition();
	StopCapture();
	s_nCntCam--;

	if (s_nCntCam <= 0)
	{
		s_nCntCam = 0;
		UninitNNCamDemoApi();
	}

    delete thd_;  

}



/*
* Intializes the hardware.
* Required by the MM::Device API.
* Typically we access and initialize hardware at this point.
* Device properties are typically created here as well, except
* the ones we need to use for defining initialization parameters.
* Such pre-initialization properties are created in the constructor.
* (This device does not have any pre-initialization properties)
*/
int CNnCameraDemo::Initialize()
{
   
   //g_hcamT = g_hcam;
   //g_pImageDataT = g_pImageData;
   //g_totalT= 0;
	g_total = 0 ;
	snapFlag_ = 0;
	snapCount = 0;
	mbiBitCount = 8;
    nBandNum = 1;
    nBPB = 1;


   string log = " CNnCameraDemo  Initialize";
   CUtils::cameraLog(log);
   if (initialized_)
      return DEVICE_OK;

   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
      LogMessage(NoHubError);

       // init camera api
    // -----------------
    int nRet = InitNNCamDemoApi();
    if (DEVICE_OK != nRet)
    {
        return nRet;
    }

	// Name
    nRet = CreateStringProperty(MM::g_Keyword_Name, g_NNDemoDeviceName, true);
    if (DEVICE_OK != nRet)
        return nRet;

	// Description
    nRet = CreateStringProperty(MM::g_Keyword_Description, "NNDEMOCSEN Camera Device Adapter", true);
    if (DEVICE_OK != nRet)
        return nRet;

   // CameraName
   nRet = CreateStringProperty(MM::g_Keyword_CameraName, "NNDemoCamera-MultiMode", true);
   assert(nRet == DEVICE_OK);

    // CameraID
    nRet = CreateProperty(MM::g_Keyword_CameraID, "V1.0", MM::String, true);
    assert(nRet == DEVICE_OK);

	 // binning
    CPropertyAction *pAct = new CPropertyAction (this, &CNnCameraDemo::OnBinning);
    nRet = CreateProperty(MM::g_Keyword_Binning, "", MM::String, false, pAct);
    assert(nRet == DEVICE_OK);

    nRet = SetAllowedBinning();
    if (nRet != DEVICE_OK)
        return nRet;


   // pixel type
   vector<string> pixelTypeValues;
   pAct = new CPropertyAction (this, &CNnCameraDemo::OnPixelType);
   //nRet = CreateStringProperty(MM::g_Keyword_PixelType, g_PixelType_8bit, false, pAct);
   //assert(nRet == DEVICE_OK);

   int nWidth = 0, nHeight = 0;
   Nncam_get_Size(g_hcam, &nWidth, &nHeight);
   unsigned short ucChannels = TDIBWIDTHBYTES(nWidth * mbiBitCount)/nWidth;
   log = " CNnCameraDemo  Initialize   ucChannels = "  + to_string(static_cast<long long>(ucChannels));
   CUtils::cameraLog(log);

   if (1 == ucChannels)
   {
		nRet = CreateProperty(MM::g_Keyword_PixelType, g_PixelType_8bit, MM::String, false, pAct);

		assert(nRet == DEVICE_OK);

		log = " CNnCameraDemo  Initialize   pixel type  nRet = "  + to_string(static_cast<long long>(nRet));
        CUtils::cameraLog(log);

		pixelTypeValues.push_back(g_PixelType_8bit);
	}
   else if (3 == ucChannels)
   {
		nRet = CreateProperty(MM::g_Keyword_PixelType, g_PixelType_24bitRGB, MM::String, false, pAct);

		assert(nRet == DEVICE_OK);

		log = " CNnCameraDemo  Initialize   pixel type  nRet = "  + to_string(static_cast<long long>(nRet));
        CUtils::cameraLog(log);

		pixelTypeValues.push_back(g_PixelType_24bitRGB);
	}
   else if(4 == ucChannels)
   {
    
	   	nRet = CreateProperty(MM::g_Keyword_PixelType, g_PixelType_32bitRGB, MM::String, false, pAct);

		assert(nRet == DEVICE_OK);

		log = " CNnCameraDemo  Initialize   pixel type  nRet = "  + to_string(static_cast<long long>(nRet));
        CUtils::cameraLog(log);

		pixelTypeValues.push_back(g_PixelType_32bitRGB);
   
   }

   nRet = SetAllowedValues(MM::g_Keyword_PixelType, pixelTypeValues);
  /*
   string pixelType;
   MM::PropertyBase* pProp ;
   pProp->Get(pixelType);
   */
    
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_PixelType, buf);
   if (ret != DEVICE_OK)
        return ret;

   std::string pixelType(buf);


   log = " CNnCameraDemo  SetAllowedValues  nRet =  " + to_string(static_cast<long long>(nRet))  + "   pixelType =  " + pixelType;
   CUtils::cameraLog(log);
   if (nRet != DEVICE_OK)
      return nRet;


   // Bit depth
   pAct = new CPropertyAction (this, &CNnCameraDemo::OnBitDepth);
   nRet = CreateIntegerProperty("BitDepth", 8, false, pAct);
   assert(nRet == DEVICE_OK);

   vector<string> bitDepths;
   bitDepths.push_back("8");
   bitDepths.push_back("10");
   bitDepths.push_back("12");
   bitDepths.push_back("14");
   bitDepths.push_back("16");
   bitDepths.push_back("32");
   nRet = SetAllowedValues("BitDepth", bitDepths);
   if (nRet != DEVICE_OK)
      return nRet;


   // Exposure
   pAct = new CPropertyAction (this, &CNnCameraDemo::OnExposure);
   nRet = CreateProperty(MM::g_Keyword_Exposure, "10.0", MM::Float, false, pAct);
   assert(nRet == DEVICE_OK);

   pAct = new CPropertyAction (this, &CNnCameraDemo::OnGlobalGain);
   nRet = CreateProperty(g_PropNameGain, "1", MM::Integer, false, pAct);
   assert(nRet == DEVICE_OK);

   SetPropertyLimits(g_PropNameGain, minGain, maxGain);


    // Auto Exposure
    int bAutoExposure = 0;
	HRESULT hr =  Nncam_get_AutoExpoEnable(g_hcam, &bAutoExposure);
    if (SUCCEEDED(hr))
    {
        pAct = new CPropertyAction (this, &CNnCameraDemo::OnATExposure);
        nRet = CreateProperty(g_PropNameATEXP, "FALSE", MM::String, false, pAct);
        assert(nRet == DEVICE_OK);

        vector<string> ATExpValues;
        ATExpValues.push_back("FALSE");
        ATExpValues.push_back("TRUE");

        nRet = SetAllowedValues(g_PropNameATEXP, ATExpValues);
        if (nRet != DEVICE_OK)
            return nRet;
    }

	// Flip Horizontal
    int b = 0 ;
	hr = Nncam_get_HFlip(g_hcam,&b);
    if (SUCCEEDED(hr))
    {
        pAct = new CPropertyAction (this, &CNnCameraDemo::OnFlipH);
        nRet = CreateProperty(g_PropNameFLPH, "FALSE", MM::String, false, pAct);
        assert(nRet == DEVICE_OK);

        vector<string> HFlipValues;
        HFlipValues.push_back("FALSE");
        HFlipValues.push_back("TRUE");

        nRet = SetAllowedValues(g_PropNameFLPH, HFlipValues);
        if (nRet != DEVICE_OK)
            return nRet;
    }

    // Image format
    pAct = new CPropertyAction (this, &CNnCameraDemo::OnImageFormat);
	nRet = CreateStringProperty(g_PropNameIFMT, "RAW", false, pAct);
	assert(nRet == DEVICE_OK);

	AddAllowedValue(g_PropNameIFMT, g_Format_RAW);


     // synchronize all properties
    // --------------------------
    nRet = UpdateStatus();
    if (nRet != DEVICE_OK)
        return nRet;

    // setup the buffer
   // ----------------
  
   nRet = ResizeImageBuffer();
   if (nRet != DEVICE_OK)
      return nRet;
   
      // initialize image buffer
    nRet = StartCapture();
    if (nRet != DEVICE_OK)
        return nRet;
	

#ifdef TESTRESOURCELOCKING
    TestResourceLocking(true);
    LogMessage("TestResourceLocking OK",true);
#endif

    initialized_ = true;

    // initialize image buffer
    GenerateEmptyImage(img_);
	log = " CNnCameraDemo  [GenerateEmptyImage]:Success!";
    CUtils::cameraLog(log);

	// initialize image buffer
	nRet = StopCapture();
	if (nRet != DEVICE_OK)
		return nRet;

	log = " CNnCameraDemo  [Initialize]:Success!";
    CUtils::cameraLog(log);

	return DEVICE_OK;
}


void CNnCameraDemo::TestResourceLocking(const bool recurse)
{
	string log = " CNnCameraDemo  TestResourceLocking";
    CUtils::cameraLog(log);
    if(recurse)
        TestResourceLocking(false);
}


void CNnCameraDemo::GenerateEmptyImage(ImgBuffer& img)
{
   string log = " CNnCameraDemo  GenerateEmptyImage   img.Height() = " + to_string(static_cast<long long>(img.Height())) 
	   + "    img.Width() =  "  +  to_string(static_cast<long long>(img.Width())) 
	   + "  img.Depth() =  " + to_string(static_cast<long long>(img.Depth()));
   CUtils::cameraLog(log);
   MMThreadGuard g(imgPixelsLock_);
   if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
      return;

   unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
   memset(pBuf, 0, img.Height()*img.Width()*img.Depth());
}


/*
* Performs exposure and grabs a single image.
* This function should block during the actual exposure and return immediately afterwards 
* (i.e., before readout).  This behavior is needed for proper synchronization with the shutter.
* Required by the MM::Camera API.
*/
int CNnCameraDemo::SnapImage()
{
   
   string log = " CNnCameraDemo  SnapImage";
   CUtils::cameraLog(log);
   static int callCounter = 0;
   ++callCounter;
   snapFlag_ = 1 ;
   g_total = 0;

   MM::MMTime startTime = GetCurrentMMTime();
    
   double exp = GetExposure();
   if (sequenceRunning_) 
    {
		// Change the exposure time
        exp = GetSequenceExposure();
        Nncam_put_ExpoTime(g_hcam, exp);
		log = " CNnCameraDemo  SnapImage  Nncam_put_ExpoTime Exp = " +  to_string(static_cast<long long>(exp));
        CUtils::cameraLog(log);
    }

	if (!m_bLiving)  // should never happen, but just in case...
	{
		StartCapture();
		log = " CNnCameraDemo  SnapImage Start Capture ";
        CUtils::cameraLog(log);
	}

	int val = 0;
	Nncam_get_Option(g_hcam, NNCAM_OPTION_TRIGGER, &val);

   if (val == 1)
	{
		int nCnt = 0;
		int nRet = DEVICE_ERR;
		log = " CNnCameraDemo  SnapImage SOFTWARE ";
        CUtils::cameraLog(log);
		do 
		{
			Nncam_Trigger(g_hcam, 1);
			nRet = WaitForFrame(img_);
			nCnt++;
		} while (DEVICE_OK != nRet && nCnt < 2);
	}
	else
	{
		if (!fastImage_)
		{	
			MM::MMTime s0(0,0);
			if( s0 < startTime )
			{
				CDeviceUtils::SleepMs((long) exp);
			}
			//TUDBG_PRINTF("SnapImage fastImage_ = %d",fastImage_);
			log = " CNnCameraDemo  SnapImage fastImage_ =  " + to_string(static_cast<long long>(fastImage_));
            CUtils::cameraLog(log);
			WaitForFrame(img_);
		}
	}

   readoutStartTime_ = GetCurrentMMTime();

   return DEVICE_OK;
}

int CNnCameraDemo::CopyToFrame(ImgBuffer& img,int nWidth,int nHeight,int ucChannels)
{

		string log = " CNnCameraDemo  CopyToFrame  nWidth  =  "+ to_string(static_cast<long long>(nWidth)) 
			+ "  nHeight = "+ to_string(static_cast<long long>(nHeight)) +  "  img.Height() =  "  + to_string(static_cast<long long>(img.Height())) 
			+  "  img.Width() =  "  + to_string(static_cast<long long>(img.Width())) 
			+ "   img.Depth() =  "  + to_string(static_cast<long long>(img.Depth())) ;
        CUtils::cameraLog(log);
//         char sz[256] = {0};
//         sprintf(sz, "[TUCAM_Buf_WaitForFrame]:%d, %d, %d, %d\n", m_frame.usWidth, m_frame.usHeight, m_frame.ucElemBytes, m_frame.ucChannels);
//         OutputDebugString(sz);

        if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
            return DEVICE_OUT_OF_MEMORY;

        int nWid = nWidth;
        int nHei = nHeight;
        int nPix = nWid * nHei*ucChannels;

//#ifdef _WIN64
        /*
        if (2 == m_frame.ucElemBytes)
        {
			*/
        /*    
		if (3 == m_frame.ucChannels)
            {
				*/
#ifdef _WIN64
               // unsigned short* pSrc = (unsigned short *)(g_pImageDataT);
				unsigned short* pSrc = (unsigned short *)(g_pImageData);
                unsigned short* pDst = (unsigned short *)(img.GetPixelsRW());

				log = " CNnCameraDemo  CopyToFrame  _WIN64  nPix = "  + to_string(static_cast<long long>(nPix));
                CUtils::cameraLog(log);
				/*
                for (int i=0; i<nPix; ++i) 
                {

				    log = " CNnCameraDemo  CopyToFrame  _WIN64  i = "  + to_string(static_cast<long long>(i));
                    CUtils::cameraLog(log);
                    *pDst++ = *pSrc++;
                    *pDst++ = *pSrc++;
                    *pDst++ = *pSrc++;
                    *pDst++ = 0;
                }
				*/

				memcpy(pDst, pSrc, nPix);

#else
                
		        log = " CNnCameraDemo  CopyToFrame    nPix = "  + to_string(static_cast<long long>(nPix));
                CUtils::cameraLog(log);
		        unsigned short* pSrc = (unsigned short *)(g_pImageData);
                unsigned char* pDst  = (unsigned char *)(img.GetPixelsRW());

				/*
                for (int i=0; i<nPix; ++i) 
                {
                    *pDst++ = (*pSrc++) >> 8;
                    *pDst++ = (*pSrc++) >> 8;
                    *pDst++ = (*pSrc++) >> 8;
                    *pDst++ = 0;
                }
				*/
				memcpy(pDst, pSrc, nPix);
#endif
           /*
			}
            else
            {
                unsigned short* pSrc = (unsigned short *)(m_frame.pBuffer + m_frame.usHeader);
                unsigned short* pDst = (unsigned short *)(img.GetPixelsRW());

                memcpy(pDst, pSrc, m_frame.uiImgSize);
            }   
			*/
        //}
       
		/*
		else
        {
            unsigned char* pSrc = (unsigned char *)(m_frame.pBuffer + m_frame.usHeader);
            unsigned char* pDst = (unsigned char *)(img.GetPixelsRW());

            if (3 == m_frame.ucChannels)
            {
                for (int i=0; i<nPix; ++i) 
                {
                    *pDst++ = *pSrc++;
                    *pDst++ = *pSrc++;
                    *pDst++ = *pSrc++;
                    *pDst++ = 0;
                }
            }
            else
            {
                memcpy(pDst, pSrc, m_frame.uiImgSize);
            }  
        }
		*/

     

        return DEVICE_OK;
    

}

int CNnCameraDemo::WaitForFrame(ImgBuffer& img)
{
	string log = " CNnCameraDemo  WaitForFrame";
    CUtils::cameraLog(log);
    MMThreadGuard g(imgPixelsLock_);
	if (g_hcam == NULL)
	{
		log = " CNnCameraDemo  DEVICE_NOT_CONNECTED ";
        CUtils::cameraLog(log);
		return DEVICE_NOT_CONNECTED ;
	}

    Nncam_put_Option(g_hcam, NNCAM_OPTION_RAW, 0);  // Set usual format
  
	int nWidth = 0, nHeight = 0;
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);
	unsigned short ucChannels = TDIBWIDTHBYTES(nWidth * mbiBitCount)/nWidth;
    if (FAILED(hr))
	{
		
		printf("failed to get size, hr = %08x\n", hr);
	    CUtils::cameraLog("failed to get size, hr = "+ hr);
    }

	CopyToFrame(img,nWidth,nHeight,ucChannels);

    return DEVICE_NATIVE_MODULE_FAILED;
}



/*
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CNnCameraDemo::GetExposure() const
{
	string log = " CNnCameraDemo  GetExposure";
    CUtils::cameraLog(log);
    char buf[MM::MaxStrLength];
    int ret = GetProperty(MM::g_Keyword_Exposure, buf);
    if (ret != DEVICE_OK)
        return 0.0;

    return atof(buf);
}


/*
 * Returns the current exposure from a sequence and increases the sequence counter
 * Used for exposure sequences
 */
double CNnCameraDemo::GetSequenceExposure() 
{
	string log = " CNnCameraDemo  GetSequenceExposure";
    CUtils::cameraLog(log);
    if (exposureSequence_.size() == 0) 
        return this->GetExposure();

    double exposure = exposureSequence_[sequenceIndex_];

    sequenceIndex_++;
    if (sequenceIndex_ >= exposureSequence_.size())
        sequenceIndex_ = 0;

    return exposure;
}

/*
* Obtains device name.
* Required by the MM::Device API.
*/
void CNnCameraDemo::GetName(char* name) const
{
	string log = " CNnCameraDemo  GetName";
    CUtils::cameraLog(log);
    // Return the name used to referr to this device adapte
    CDeviceUtils::CopyLimitedString(name, g_NNDemoDeviceName);
}

/*
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int CNnCameraDemo::Shutdown()
{
    // Close the get value of temperature thread

    string log = " CNnCameraDemo  Shutdown";
    CUtils::cameraLog(log);

    if (NULL != m_hThdTempEvt)
    {
        m_bTemping = false;
        WaitForSingleObject(m_hThdTempEvt, INFINITE);	
        CloseHandle(m_hThdTempEvt);
        m_hThdTempEvt = NULL;
    }

    StopSequenceAcquisition();

    UninitNNCamDemoApi();
    initialized_ = false;
    return DEVICE_OK;
}



/*                                                                    
* Stop Capture and wait for the Sequence thread finished                                   
*/                                                                        
int CNnCameraDemo::StopSequenceAcquisition()                                     
{
     string log = " CNnCameraDemo  StopSequenceAcquisition";
     CUtils::cameraLog(log);

     if (thd_->IsStopped() && !returnToSoftwareTriggers_)
        return DEVICE_OK;

    m_bLiving = false;
	snapFlag_ = 4 ;

    if (NULL == g_hcam)
    {
            return DEVICE_NOT_CONNECTED;
    }
    thd_->Stop(); 
    Nncam_Pause(g_hcam,true); 
    thd_->wait();

    Nncam_Stop(g_hcam);                      // Stop capture   
    ReleaseBuffer();

    // Switch back to software trigger mode if that is what the user used
    int val = 0;
	Nncam_get_Option(g_hcam, NNCAM_OPTION_TRIGGER, &val);
	if (val == 2)
    {
       Nncam_Trigger(g_hcam, 0);
       returnToSoftwareTriggers_ = false;
    }
	else if(val == 1)
	{
	   Nncam_Trigger(g_hcam, 1);
	}

    StartCapture();  // avoid wasting time in the SnapImage function 

    return DEVICE_OK;
} 



/*
* Handles "IamgeFormat" property.
*/
int CNnCameraDemo::OnImageFormat(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    

    string	log = " CNnCameraDemo  OnImageFormat!";
    CUtils::cameraLog(log);
    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            string val;
            pProp->Get(val);

            if (val.length() != 0)
            {
                if (0 == val.compare(g_Format_RAW))
                {

                }  

                char szPath[MAX_PATH];
                GetCurrentDirectory(MAX_PATH, szPath);
                strcat(szPath, g_FileName);

                //OutputDebugString(szPath);

                // Create not exists folder
                //if (!PathIsDirectory(szPath))
                    
				CreateDirectory(szPath, NULL);

				string szPath1 =  szPath ;
				log = " CNnCameraDemo  OnImageFormat!  szPath =  " + szPath1;
                CUtils::cameraLog(log);

                SYSTEMTIME sysTm;
                GetLocalTime(&sysTm);
                sprintf(m_szImgPath, ("%s\\MM_%02d%02d%02d%02d%03d"), szPath, sysTm.wDay, sysTm.wHour, sysTm.wMinute, sysTm.wSecond, sysTm.wMilliseconds);

                m_bSaving = true;

                //OutputDebugString(m_szImgPath);
            }

            ret = DEVICE_OK;
        }
        break;
    case MM::BeforeGet:
        {
            pProp->Set(g_Format_RAW);

            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }

    return ret;
}


bool CNnCameraDemo::SaveRaw(char *pfileName, unsigned char *pData, unsigned long ulSize)
{
    FILE *pfile = NULL;
    string szPath = pfileName;
    szPath += ".raw";

    //OutputDebugString(szPath.c_str());

	string	log = " CNnCameraDemo  SaveRaw!   szPath =  " + szPath;
    CUtils::cameraLog(log);

    pfile = fopen(szPath.c_str(), "wb");

    if(NULL != pfile) 
    {
        fwrite(pData, 1, ulSize, pfile);
        fclose(pfile);

//        delete pfile;

        pfile = NULL;
        //OutputDebugString("[SaveRaw]:NULL!\n");
		log = " CNnCameraDemo  SaveRaw  NULL! " ;
        CUtils::cameraLog(log);


        return true;
    }

    return false;
}

/*
* Handles "Exposure" property.
*/
int CNnCameraDemo::OnExposure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   
	string	log = " CNnCameraDemo  OnExposure!";
    CUtils::cameraLog(log);
    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            double dblExp;
            pProp->Get(dblExp);          

			dblExp = dblExp * 1000;
            if (dblExp < exposureMinimum_)
            {
               dblExp = exposureMinimum_;
            }
            else if (dblExp > exposureMaximum_) {
               dblExp = exposureMaximum_;
            }
			unsigned dblExp1 = dblExp;
         
			HRESULT hr = Nncam_put_ExpoTime(g_hcam,dblExp);
			if (FAILED(hr))
	        {
		
	      	   log = "CNnCameraDemo OnExposure!  AfterSet  Nncam_put_ExpoTime failed .";
	           CUtils::cameraLog(log);
            }
           else
          {
			 log = " CNnCameraDemo  OnExposure! AfterSet  Nncam_put_ExpoTime success !  dblExp =  "+  to_string(static_cast<double long>(dblExp))
				 + "  exposureMinimum_ =  " + to_string(static_cast<long long>(exposureMinimum_)) + "  exposureMaximum_ =  " + to_string(static_cast<long long>(exposureMaximum_));
             CUtils::cameraLog(log);
			
			}


            ret = DEVICE_OK;
        }
        break;
    case MM::BeforeGet:
        {
            double dblExp = 0.0f;
			unsigned dblExp1 = dblExp;

			double dblExpTest;
            pProp->Get(dblExpTest);    
			//dblExpTest =  dblExpTest;
			dblExp = dblExpTest ;
			log = " CNnCameraDemo  OnExposure!   success ! BeforeGet dblExp1 =  "+  to_string(static_cast<double long>(dblExp1))
				+"  dblExpTest =  " + to_string(static_cast<double long>(dblExpTest));
            CUtils::cameraLog(log);

            dblExpTest = dblExpTest * 1000 ;
			if (dblExpTest < exposureMinimum_)
            {
               dblExpTest = exposureMinimum_;
            }
            else if (dblExpTest > exposureMaximum_) {
               dblExpTest = exposureMaximum_;
            }
			HRESULT hr = Nncam_put_ExpoTime(g_hcam,dblExpTest);
			if (FAILED(hr))
	        {
		
	      	   log = "CNnCameraDemo OnExposure!  BeforeGet  Nncam_put_ExpoTime failed .";
	           CUtils::cameraLog(log);
            }
           else
          {
			 log = " CNnCameraDemo  OnExposure!  BeforeGet Nncam_put_ExpoTime success !  dblExpTest =  "+  to_string(static_cast<double long>(dblExpTest))
             + " dblExp =  "  + to_string(static_cast<double long>(dblExp));
			 CUtils::cameraLog(log);
			
			}

			hr = Nncam_get_ExpoTime(g_hcam,&dblExp1);
            //dblExp = dblExp1 ;
			
			pProp->Set(dblExp);
			if (FAILED(hr))
	        {
		
	      	   log = "CNnCameraDemo OnExposure! BeforeGet  Nncam_get_ExpoTime failed .";
	           CUtils::cameraLog(log);
            }
           else
          {
			 log = " CNnCameraDemo  OnExposure! BeforeGet  Nncam_get_ExpoTime success !  dblExp1 =  "+  to_string(static_cast<double long>(dblExp1));
             CUtils::cameraLog(log);
			
			}



            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }


	log = " CNnCameraDemo  OnExposure!  ret =  "+  to_string(static_cast<long long>(ret));
    CUtils::cameraLog(log);
    return ret;
}


/*
* Handles "GlobalGain" property.
*/
int CNnCameraDemo::OnGlobalGain(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	string 	log = " CNnCameraDemo  OnGlobalGain! ";
    CUtils::cameraLog(log);

    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            double dblGain;
            pProp->Get(dblGain);          

			log = " CNnCameraDemo  OnGlobalGain! Before  Nncam_put_ExpoAGain !  dblGain =  "+  to_string(static_cast<long long>(dblGain));
            CUtils::cameraLog(log);
            HRESULT hr = Nncam_put_ExpoAGain(g_hcam,  dblGain);
			if (FAILED(hr))
	        {
		
	      	   log = "CNnCameraDemo OnGlobalGain!  Nncam_put_ExpoAGain failed .";
	           CUtils::cameraLog(log);
            }
           else
           {
			 log = " CNnCameraDemo  OnGlobalGain!  Nncam_put_ExpoAGain success !  dblGain =  "+  to_string(static_cast<long long>(dblGain));
             CUtils::cameraLog(log);
			
			}

            ret = DEVICE_OK;
        }
        break;
    case MM::BeforeGet:
        {
            double dblGain = 0.0f;
			unsigned short dblGain1 = 0;

            HRESULT hr =  Nncam_get_ExpoAGain(g_hcam,  &dblGain1);
			if (FAILED(hr))
	        {
		
	      	   log = "CNnCameraDemo OnGlobalGain!  Nncam_get_ExpoAGain failed .";
	           CUtils::cameraLog(log);
            }
           else
           {
			 log = " CNnCameraDemo  OnGlobalGain!  Nncam_get_ExpoAGain success !  dblGain1 =  "+  to_string(static_cast<long long>(dblGain1));
             CUtils::cameraLog(log);
			
			}

            dblGain =  (double)dblGain1 ;
			pProp->Set(dblGain);

            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }

    return ret;
}


/*
* Handles "ATExposure" property.
*/
int CNnCameraDemo::OnATExposure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
  
    string 	log = " CNnCameraDemo  OnATExposure! ";
    CUtils::cameraLog(log);
    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            string val;
            pProp->Get(val);
			log = " CNnCameraDemo  OnATExposure! AfterSet  val = " + val;
            CUtils::cameraLog(log);

            if (val.length() != 0)
            {
                int bAutoExposure = 0;
	            HRESULT hr =  Nncam_get_AutoExpoEnable(g_hcam, &bAutoExposure);

                if (SUCCEEDED(hr))
                {
					log = " CNnCameraDemo  OnATExposure! AfterSet  Nncam_get_AutoExpoEnable   success  bAutoExposure = " +  to_string(static_cast<long long>(bAutoExposure));
                    CUtils::cameraLog(log);
                    if (0 == val.compare("TRUE"))
                    {
                        Nncam_put_AutoExpoEnable(g_hcam, 1);
						log = " CNnCameraDemo  OnATExposure! AfterSet  Nncam_put_AutoExpoEnable  1";
                        CUtils::cameraLog(log);
                    }
                    else
                    {
                        Nncam_put_AutoExpoEnable(g_hcam, 0);
					    log = " CNnCameraDemo  OnATExposure! AfterSet  Nncam_put_AutoExpoEnable   0";
                        CUtils::cameraLog(log);
                    }
                }
				else
				{
				
						log = " CNnCameraDemo  OnATExposure! Nncam_get_AutoExpoEnable   failed";
                        CUtils::cameraLog(log);

				}

                OnPropertyChanged(g_PropNameATEXP, val.c_str());

                ret = DEVICE_OK;
            }
        }
        break;
    case MM::BeforeGet:
        {
            string nVal;
            int bAutoExposure = 0;
	        HRESULT hr =  Nncam_get_AutoExpoEnable(g_hcam, &bAutoExposure);
			pProp->Get(nVal);

			log = " CNnCameraDemo  OnATExposure! BeforeGet  nVal = " + nVal  + "  bAutoExposure =  " + to_string(static_cast<long long>(bAutoExposure));
            CUtils::cameraLog(log);

            if (1 == bAutoExposure)
            {
                pProp->Set("TRUE");
            }
            else
            {
                pProp->Set("FALSE");
            }

            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }

    return ret;
}

/**
* Handles "PixelType" property.
*/
int CNnCameraDemo::OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct)
{

   string log = " CNnCameraDemo  OnPixelType ";
   CUtils::cameraLog(log);

   int nWidth = 0, nHeight = 0;
   Nncam_get_Size(g_hcam, &nWidth, &nHeight);
   unsigned short ucChannels = TDIBWIDTHBYTES(nWidth * mbiBitCount)/nWidth;


   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         string pixelType;
         pProp->Get(pixelType);

		 log = " CNnCameraDemo  OnPixelType  AfterSet pixelType = "  + pixelType;
         CUtils::cameraLog(log);

         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 2);
            bitDepth_ = 16;
            ret=DEVICE_OK;
         }		 
         else if ( pixelType.compare(g_PixelType_24bitRGB) == 0)
         {
            nComponents_ = 3;
            img_.Resize(img_.Width(), img_.Height(), 3);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
         {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
         {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 8);
            bitDepth_ = 16;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_32bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 32;
            ret=DEVICE_OK;
         }
         else
         {
            // on error switch to default pixel type
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            pProp->Set(g_PixelType_8bit);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
         }
      }
      break;
   case MM::BeforeGet:
      {
         long bytesPerPixel = GetImageBytesPerPixel();
		 log = " CNnCameraDemo  OnPixelType  BeforeGet bytesPerPixel = "  + to_string(static_cast<long long>(bytesPerPixel));
         CUtils::cameraLog(log);
         
		 if (bytesPerPixel == 1)
         {
         	pProp->Set(g_PixelType_8bit);
         }
         else if (bytesPerPixel == 2)
         {
         	pProp->Set(g_PixelType_16bit);
         }
		 else if (bytesPerPixel == 3)
         {
         	pProp->Set(g_PixelType_24bitRGB);
         }
         else if (bytesPerPixel == 4)
         {
            if (nComponents_ == 4)
            {
			   pProp->Set(g_PixelType_32bitRGB);
            }
            else if (nComponents_ == 1)
            {
               pProp->Set(::g_PixelType_32bit);
            }
         }
         else if (bytesPerPixel == 8)
         {
            pProp->Set(g_PixelType_64bitRGB);
         }
		 else
         {
            pProp->Set(g_PixelType_8bit);
         }

		if(ucChannels == 3)
		{
		    pProp->Set(g_PixelType_24bitRGB);
		}
		else if(ucChannels == 4)
		{
		   pProp->Set(g_PixelType_32bitRGB);
		}
		 

         ret = DEVICE_OK;
      } break;
   default:
      break;
   }
   return ret; 
}

/**
* Handles "BitDepth" property.
*/
int CNnCameraDemo::OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	
   string	log = " CNnCameraDemo  OnBitDepth!";
   CUtils::cameraLog(log);
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         long bitDepth;
         pProp->Get(bitDepth);

		 unsigned int bytesPerComponent;

		 log = " CNnCameraDemo  OnBitDepth!   bitDepth=  " + to_string(static_cast<long long>(bitDepth));
         CUtils::cameraLog(log);

         switch (bitDepth) {
            case 8:
					bytesPerComponent = 1;
               bitDepth_ = 8;
               ret=DEVICE_OK;
            break;
            case 10:
					bytesPerComponent = 2;
               bitDepth_ = 10;
               ret=DEVICE_OK;
            break;
            case 12:
					bytesPerComponent = 2;
               bitDepth_ = 12;
               ret=DEVICE_OK;
            break;
            case 14:
					bytesPerComponent = 2;
               bitDepth_ = 14;
               ret=DEVICE_OK;
            break;
            case 16:
					bytesPerComponent = 2;
               bitDepth_ = 16;
               ret=DEVICE_OK;
            break;
            case 32:
               bytesPerComponent = 4;
               bitDepth_ = 32; 
               ret=DEVICE_OK;
            break;
            default: 
               // on error switch to default pixel type
					bytesPerComponent = 1;

               pProp->Set((long)8);
               bitDepth_ = 8;
               ret = ERR_UNKNOWN_MODE;
            break;
         }
			char buf[MM::MaxStrLength];
			GetProperty(MM::g_Keyword_PixelType, buf);
			std::string pixelType(buf);
			unsigned int bytesPerPixel = 1;
			

         // automagickally change pixel type when bit depth exceeds possible value
         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
				if( 2 == bytesPerComponent)
				{
					SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);
					bytesPerPixel = 2;
				}
				else if ( 4 == bytesPerComponent)
            {
					SetProperty(MM::g_Keyword_PixelType, g_PixelType_32bit);
					bytesPerPixel = 4;

            }else
				{
				   bytesPerPixel = 1;
				}
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
				bytesPerPixel = 2;
         }
		    else if ( pixelType.compare(g_PixelType_24bitRGB) == 0)
			{
				bytesPerPixel = 3;
			}
		    else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
			{
				bytesPerPixel = 4;
			}
			else if ( pixelType.compare(g_PixelType_32bit) == 0)
			{
				bytesPerPixel = 4;
			}
			else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
			{
				bytesPerPixel = 8;
			}
			img_.Resize(img_.Width(), img_.Height(), bytesPerPixel);

      } break;
   case MM::BeforeGet:
      {
         log = " CNnCameraDemo  OnBitDepth!  BeforeGet  bitDepth_ =  " +  to_string(static_cast<long long>(bitDepth_));
         CUtils::cameraLog(log);
         pProp->Set((long)bitDepth_);
         ret=DEVICE_OK;
      } break;
   default:
      break;
   }
   log = " CNnCameraDemo  OnBitDepth!  ret =  " +  to_string(static_cast<long long>(ret));
   CUtils::cameraLog(log);
   return ret; 
}

/*
* Handles "FlipHorizontal" property.
*/
int CNnCameraDemo::OnFlipH(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	
	string	log = " CNnCameraDemo  OnFlipH!";
    CUtils::cameraLog(log);
    if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            string val;
            pProp->Get(val);
            if (val.length() != 0)
            {
				int b = 0 ;
	            HRESULT hr = Nncam_get_HFlip(g_hcam,&b);
				log = " CNnCameraDemo  OnFlipH!  AfterSet  b =  "+  to_string(static_cast<long long>(b));
                CUtils::cameraLog(log);
                if (SUCCEEDED(hr))
                {
                    if (0 == val.compare("TRUE"))
                    {
                        Nncam_put_HFlip(g_hcam, 1);
                    }
                    else
                    {
                        Nncam_put_HFlip(g_hcam, 0);
                    }
                }

                OnPropertyChanged(g_PropNameFLPH, val.c_str());

                ret = DEVICE_OK;
            }
        }
        break;
    case MM::BeforeGet:
        {
            int nVal = 0;
            
			HRESULT hr = Nncam_get_HFlip(g_hcam,&nVal);
			log = " CNnCameraDemo  OnFlipH!  BeforeGet  nVal =  "+  to_string(static_cast<long long>(nVal));
            CUtils::cameraLog(log);

            if (1 == nVal)
            {
                pProp->Set("TRUE");
            }
            else
            {
                pProp->Set("FALSE");
            }

            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }

    return ret;
}

/*
* Handles "Binning" property.
*/
int CNnCameraDemo::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   
	string	log = " CNnCameraDemo  OnBinning!";
    CUtils::cameraLog(log);
    int ret = DEVICE_ERR;
    switch(eAct)
    {
    case MM::AfterSet:
        {
            if(IsCapturing())
                return DEVICE_CAMERA_BUSY_ACQUIRING;

            // the user just set the new value for the property, so we have to
            // apply this value to the 'hardware'.

            string val;
            pProp->Get(val);
            if (val.length() != 0)
            {
                if (g_hcam)
                {
                    
					if(SUCCEEDED(Nncam_Stop(g_hcam)))      // Stop capture   
					{
						log = " CNnCameraDemo  OnBinning  Nncam_Stop success.  val  =  " + val;
                        CUtils::cameraLog(log);
						ReleaseBuffer();

                        binVal = atoi(val.c_str()) ;
						/*
                		exposureMinimum_ = 10000;
						exposureMaximum_ = 3600000;
						SetPropertyLimits(MM::g_Keyword_Exposure, exposureMinimum_, exposureMaximum_);
						*/

						StartCapture();
						ResizeImageBuffer();

						roiX_ = 0;
						roiY_ = 0;
				  }
					else
					{
					
						log = " CNnCameraDemo  OnBinning  Nncam_Stop failed.";
                        CUtils::cameraLog(log);
					
					}
                }

                OnPropertyChanged(MM::g_Keyword_Binning, val.c_str());

                ret = DEVICE_OK;
            }
        }
        break;
    case MM::BeforeGet:
        {
            long binSize_ = 100.0;

            pProp->Set(binSize_);
            ret = DEVICE_OK;
        }
        break;
    default:
        break;
    }

	log = " CNnCameraDemo  OnBinning! ret = " + to_string(static_cast<long long>(ret));
    CUtils::cameraLog(log);
    return ret; 
}

int CNnCameraDemo::OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		dropPixels_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(dropPixels_?1L:0L);
   }

   return DEVICE_OK;
}

int CNnCameraDemo::OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		fastImage_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(fastImage_?1L:0L);
   }

   return DEVICE_OK;
}

int CNnCameraDemo::OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long tvalue = 0;
      pProp->Get(tvalue);
		saturatePixels_ = (0==tvalue)?false:true;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(saturatePixels_?1L:0L);
   }

   return DEVICE_OK;
}


int CNnCameraDemo::OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      double tvalue = 0;
      pProp->Get(tvalue);
		fractionOfPixelsToDropOrSaturate_ = tvalue;
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set(fractionOfPixelsToDropOrSaturate_);
   }

   return DEVICE_OK;
}

bool CNnCameraDemo::IsCapturing() 
{
	 string log = " CNnCameraDemo  IsCapturing";
     CUtils::cameraLog(log);
     return !thd_->IsStopped();
}

static void __stdcall EventCallback(unsigned nEvent, void* pCallbackCtx)
{
    
	 string log = " CNnCameraDemo  Enter EventCallback  snapFlag_ =  " + to_string(static_cast<long long>(snapFlag_))
		 + "   g_total =  " + to_string(static_cast<long long>(g_total));
     CUtils::cameraLog(log);
	if (NNCAM_EVENT_IMAGE == nEvent)
    {
		if(snapFlag_ == 1)
		{
		    
			if( g_total == 0)
			{
				NncamFrameInfoV2 info = { 0 };
			   // HRESULT hr = Nncam_PullImageV2(g_hcam, g_pImageDataT, 24, &info);
				HRESULT hr = Nncam_PullImageV2(g_hcam, g_pImageData, mbiBitCount, &info);
				if (FAILED(hr))
				{
					printf("failed to pull image, hr = %08x\n", hr);
					log = " CNnCameraDemo  EventCallback  failed to pull image";
					CUtils::cameraLog(log);
				}
				else
				{
					/* After we get the image data, we can do anything for the data we want to do */
					printf("pull image ok, total = %u, res = %u x %u\n", ++g_total, info.width, info.height);
					printf("  g_pImageData  ==  ");
					log = " CNnCameraDemo  EventCallback  pull image ok";
					CUtils::cameraLog(log);
			
			        
					++snapCount;
					wchar_t strPath[MAX_PATH];
					string imgName = "";
					swprintf(strPath, L"%04u.jpg", snapCount);
					CImgProc::Wchar_tToString(imgName,strPath);
				
					string imgPath = "H:/projects1/testImg/"  +imgName;
					CUtils::cameraLog("imgPath =  "+ imgPath);
			        //Mat img = CImgProc::Rgb24ToMat(g_pImageData,info.height,info.width);
					Mat img = CImgProc::TransBufferToMat((unsigned char*)g_pImageData,info.width,info.height,nBandNum,nBPB);
					
					//imshow("img",img);
					imwrite(imgPath,img);
					//waitKey(1);

					snapFlag_ = 2;
					g_total = 0 ;

					/*
					wchar_t strPath[MAX_PATH];
					swprintf(strPath, L"%04u.jpg", m_nSnapFile++);
					SaveImageByWIC(strPath, pSnapData, &header);
					*/
				}
			}
	   }
	  else if(snapFlag_ == 3)
	  {
		
			NncamFrameInfoV2 info = { 0 };
		   // HRESULT hr = Nncam_PullImageV2(g_hcam, g_pImageDataT, 24, &info);
			HRESULT hr = Nncam_PullImageV2(g_hcam, g_pImageData, mbiBitCount, &info);
			if (FAILED(hr))
			{
				printf("failed to pull image, hr = %08x\n", hr);
				log = " CNnCameraDemo  EventCallback  failed to pull image";
				CUtils::cameraLog(log);
			}
			else
			{
				/* After we get the image data, we can do anything for the data we want to do */
				printf("pull image ok, total = %u, res = %u x %u\n", ++g_total, info.width, info.height);
				printf("  g_pImageData  ==  ");
				log = " CNnCameraDemo  EventCallback  pull image ok";
				CUtils::cameraLog(log);
			
			
				wchar_t strPath[MAX_PATH];
				string imgName = "";
				swprintf(strPath, L"%04u.jpg", g_total);
				CImgProc::Wchar_tToString(imgName,strPath);
				
				string imgPath = "H:/projects1/testImg/" + imgName;
				CUtils::cameraLog("imgPath =  "+ imgPath);
				//Mat img = CImgProc::Rgb24ToMat(g_pImageData,info.height,info.width);
			    Mat img = CImgProc::TransBufferToMat((unsigned char*)g_pImageData,info.width,info.height,nBandNum,nBPB);
				//imshow("img",img);
				imwrite(imgPath,img);
				//waitKey(1);	

				/*
				wchar_t strPath[MAX_PATH];
				swprintf(strPath, L"%04u.jpg", m_nSnapFile++);
				SaveImageByWIC(strPath, pSnapData, &header);
				*/
			}
	   }
    }
    else
    {
        printf("event callback: %d\n", nEvent);
		log = " CNnCameraDemo  event callback! " + to_string(static_cast<long long>(nEvent));
        CUtils::cameraLog(log);
    }
}

int CNnCameraDemo::StopCapture()
{
    m_bLiving = false;

    //TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam);                 // If you called TUCAM_Buf_WaitForFrames()
	string	log = " CNnCameraDemo  StopCapture! ";
    CUtils::cameraLog(log);
    HRESULT hr = Nncam_Stop(g_hcam);                      // Stop capture   
    if (FAILED(hr))
	{
	  	log = " CNnCameraDemo  Nncam_Stop failed !  hr = " + to_string(static_cast<long long>(hr));
        CUtils::cameraLog(log);
	
	}
	else
	{
	
	    log = " CNnCameraDemo  Nncam_Stop success !  hr = " + to_string(static_cast<long long>(hr));
        CUtils::cameraLog(log);
	
	}


	ReleaseBuffer();

    return DEVICE_OK;

}

int CNnCameraDemo::StartCapture()
{

	string	log = " CNnCameraDemo  StartCapture! ";
    CUtils::cameraLog(log);
	memset(&m_header, 0, sizeof(m_header));
	m_header.biSize = sizeof(BITMAPINFOHEADER);
	m_header.biPlanes = 1;
	m_header.biBitCount = mbiBitCount;

	int nWidth = 0, nHeight = 0;
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);
	if (binVal > 0)
	{
		if(binVal == 1)
		{
			Nncam_put_eSize(g_hcam,0);
		}
		else
		{
		     nWidth = nWidth / binVal ;
	         nHeight = nHeight / binVal ;
	         Nncam_put_Size(g_hcam,nWidth,nHeight);
		}

	}
	
    //Nncam_put_eSize(g_hcam,binVal);
	if (SUCCEEDED(Nncam_get_Size(g_hcam, (int*)&m_header.biWidth, (int*)&m_header.biHeight)))
	{
		
		m_header.biSizeImage = TDIBWIDTHBYTES(m_header.biWidth * m_header.biBitCount) * m_header.biHeight;
		log = " CNnCameraDemo  StartCapture!   m_header.biWidth =  " + to_string(static_cast<long long>(m_header.biWidth))
		   + 	"   m_header.biHeight =   "  + to_string(static_cast<long long>(m_header.biHeight))
		   + " m_header.biSizeImage =  " + to_string(static_cast<long long>(m_header.biSizeImage))+ "   binVal =   " + to_string(static_cast<long long>(binVal));
        CUtils::cameraLog(log);
		if (g_pImageData)
		{
			free(g_pImageData);
			g_pImageData = NULL;
		}
		   
			
		int nRet = AllocBuffer();
		if (nRet != DEVICE_OK)
		{
			return nRet;
		}
		
		// Start capture
	 /*
		HWND  m_hWnd = FindWindow(L"Start capture", 0);
       Nncam_StartPullModeWithWndMsg(g_hcam, m_hWnd, MSG_CAMEVENT);
		*/


		//Nncam_put_Size(g_hcamT,m_header.biWidth,m_header.biHeight);
		Nncam_put_Size(g_hcam,m_header.biWidth,m_header.biHeight);
		//g_hcamT = g_hcam;
		//g_pImageDataT = g_pImageData;
		//HRESULT hr = Nncam_StartPullModeWithCallback(g_hcamT, EventCallback, NULL);
		HRESULT hr = Nncam_StartPullModeWithCallback(g_hcam, EventCallback, NULL);
        if (FAILED(hr))
		{
		    string log = " CNnCameraDemo  StartCapture failed to start camera  snapFlag_ = " + to_string(static_cast<long long>(snapFlag_));
            CUtils::cameraLog(log);
			printf("failed to start camera, hr = %08x\n", hr);
			if (snapFlag_ == 2 || 0 == snapFlag_ || 1 == snapFlag_ || 3 == snapFlag_ || 4 == snapFlag_)
			{
			  RestartCapture();
			}
			
			return nRet;
		}
        else
        {

			string log = " CNnCameraDemo  StartCapture success to start camera";
			CUtils::cameraLog(log);
			printf("press any key to exit\n");
			
			
			


			return DEVICE_OK;
            //getc(stdin);
        }

	}


    return DEVICE_ERR;


}


int CNnCameraDemo::RestartCapture()
{
	Nncam_Pause(g_hcam,true); 
	Nncam_Stop(g_hcam);                      // Stop capture   
    ReleaseBuffer();

    // Switch back to software trigger mode if that is what the user used
    int val = 0;
	Nncam_get_Option(g_hcam, NNCAM_OPTION_TRIGGER, &val);
	if (val == 2)
    {
       Nncam_Trigger(g_hcam, 0);
       returnToSoftwareTriggers_ = false;
    }
	else if(val == 1)
	{
	   Nncam_Trigger(g_hcam, 1);
	}

    //StartCapture();  // avoid wasting time in the SnapImage function 


	string	log = " CNnCameraDemo  RestartCapture! ";
    CUtils::cameraLog(log);
	memset(&m_header, 0, sizeof(m_header));
	m_header.biSize = sizeof(BITMAPINFOHEADER);
	m_header.biPlanes = 1;
	m_header.biBitCount = mbiBitCount;


	int nWidth = 0, nHeight = 0;
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);
	if (binVal > 0)
	{
		if(binVal == 1)
		{
		  Nncam_put_eSize(g_hcam,0);
		}
		else
		{
		    nWidth = nWidth / binVal ;
	        nHeight = nHeight / binVal ;
	        Nncam_put_Size(g_hcam,nWidth,nHeight);
		}
	}
    //Nncam_put_eSize(g_hcam,binVal);
	if (SUCCEEDED(Nncam_get_Size(g_hcam, (int*)&m_header.biWidth, (int*)&m_header.biHeight)))
	{
		
		m_header.biSizeImage = TDIBWIDTHBYTES(m_header.biWidth * m_header.biBitCount) * m_header.biHeight;
		log = " CNnCameraDemo  RestartCapture!   m_header.biWidth =  " + to_string(static_cast<long long>(m_header.biWidth))
		   + 	"   m_header.biHeight =   "  + to_string(static_cast<long long>(m_header.biHeight))
		   + " m_header.biSizeImage =  " + to_string(static_cast<long long>(m_header.biSizeImage)) + "   binVal =   " + to_string(static_cast<long long>(binVal));
        CUtils::cameraLog(log);
		if (g_pImageData)
		{
			free(g_pImageData);
			g_pImageData = NULL;
		}
		   
			
		int nRet = AllocBuffer();
		if (nRet != DEVICE_OK)
		{
			return nRet;
		}
		
		// Start capture
	 
		Nncam_put_Size(g_hcam,m_header.biWidth,m_header.biHeight);
		
		HRESULT hr = Nncam_StartPullModeWithCallback(g_hcam, EventCallback, NULL);
        if (FAILED(hr))
		{
		    string log = " CNnCameraDemo  StartCapture failed to start camera";
            CUtils::cameraLog(log);
			printf("failed to start camera, hr = %08x\n", hr);
			return nRet;
		}
        else
        {

			
			//RestartCapture();
			string log = " CNnCameraDemo  RestartCapture success to start camera";
			CUtils::cameraLog(log);
			printf("press any key to exit\n");
			

			return DEVICE_OK;
            //getc(stdin);
        }

	}

}

/*
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CNnCameraDemo::GetImageBufferSize() const
{
/*
    if (NULL != m_frame.pBuffer)
    {
        return (m_frame.usWidth * m_frame.usHeight * GetImageBytesPerPixel());
    }

    return 0;
*/    
    string	log = " CNnCameraDemo  GetImageBufferSize!  img_.Width() =  " + to_string(static_cast<long long>(img_.Width()));
    CUtils::cameraLog(log);
    return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

/*
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHight() and GetImageBytesPerPixel().
* The calling program allso assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* CNnCameraDemo::GetImageBuffer()
{
	string log = " CNnCameraDemo  GetImageBuffer";
    CUtils::cameraLog(log);
    MMThreadGuard g(imgPixelsLock_);
    MM::MMTime readoutTime(readoutUs_);
    while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}		
    unsigned char *pB = (unsigned char*)(img_.GetPixels());
    return pB;  //NULL
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CNnCameraDemo::GetImageWidth() const
{
   string log = " CNnCameraDemo  GetImageWidth  img_.Width() =  "  + to_string(static_cast<long long>(img_.Width()));
   CUtils::cameraLog(log);
   return img_.Width();
}

/*
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CNnCameraDemo::GetImageHeight() const
{
/*
    if (NULL != m_frame.pBuffer)
    {
        return m_frame.usHeight;
    }

    return 0;
*/
	string	log = " CNnCameraDemo  GetImageHeight!  img_.Height() =  "    + to_string(static_cast<long long>(img_.Height()));
    CUtils::cameraLog(log);
    return img_.Height();
}

/*
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CNnCameraDemo::GetImageBytesPerPixel() const
{
/*
    if (NULL != m_frame.pBuffer)
    {
        int nChnnels = (1 == m_frame.ucChannels) ? 1 : 4;

        return (m_frame.ucElemBytes * nChnnels);
    }

    return 1;
*/

	string	log = " CNnCameraDemo  GetImageBytesPerPixel!   img_.Depth() =  "   + to_string(static_cast<long long>(img_.Depth()));
    CUtils::cameraLog(log);
    return img_.Depth();
} 

/*
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CNnCameraDemo::GetBitDepth() const
{
/*
    if (NULL != m_frame.pBuffer)
    {
        return (1 == m_frame.ucElemBytes) ? 8 : 16;
    }

    return 8;
*/
	string	log = " CNnCameraDemo  GetBitDepth!  bitDepth_ =  "  + to_string(static_cast<long long>(bitDepth_));
    CUtils::cameraLog(log);
    return bitDepth_;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int CNnCameraDemo::GetBinning() const
{
   string	log = " CNnCameraDemo  GetBinning! ";
   CUtils::cameraLog(log);
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Binning, buf);
   if (ret != DEVICE_OK)
      return 1;
   return atoi(buf);
}

/**
* Sets binning factor.
* Required by the MM::Camera API.
*/
int CNnCameraDemo::SetBinning(int binF)
{

   string	log = " CNnCameraDemo  SetBinning! ";
   CUtils::cameraLog(log);
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

/*
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void CNnCameraDemo::SetExposure(double exp)
{

	string	log = " CNnCameraDemo  SetExposure!   exp =  " + to_string(static_cast<double long>(exp));
    CUtils::cameraLog(log);
    exp =  exp * 1000;
	if (exp < exposureMinimum_)
    {
       exp = exposureMinimum_;
    } else if (exp > exposureMaximum_) {
       exp = exposureMaximum_;
    }
	exp = exp / 1000.0;
    SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
    GetCoreCallback()->OnExposureChanged(this, exp);
}

/*
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but should try do as close as possible.
* If the hardware does not have this capability the software should simulate the ROI by
* appropriately cropping each frame.
* This demo implementation ignores the position coordinates and just crops the buffer.
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/

int CNnCameraDemo::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{

	string	log = " CNnCameraDemo  SetROI! ";
    CUtils::cameraLog(log);
    if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

    if (xSize == 0 && ySize == 0)
    {
        // effectively clear ROI
        ResizeImageBuffer();
        roiX_ = 0;
        roiY_ = 0;
        m_bROI = false;
    }
    else
    {
        if (NULL == g_hcam)
            return DEVICE_NOT_CONNECTED;

        m_bLiving = false;
        HRESULT hr = Nncam_Stop(g_hcam);      // Stop capture   
        if(FAILED(hr))
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_Stop failed ";
          CUtils::cameraLog(log);
		
		}
		else
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_Stop success ";
          CUtils::cameraLog(log);
		
		}

		ReleaseBuffer();

        hr = Nncam_put_Roi(g_hcam,  x,  y,  xSize,  ySize);
        if(FAILED(hr))
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_put_Roi failed ";
          CUtils::cameraLog(log);
		}
		else
		{
		
		  log = " CNnCameraDemo  SetROI!  Nncam_put_Roi success ";
          CUtils::cameraLog(log);
		}
		 hr = Nncam_get_Roi(g_hcam,  &x,  &y,  &xSize,  &ySize);
		if(FAILED(hr))
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_get_Roi failed ";
          CUtils::cameraLog(log);
		}
		else
		{
		
		  log = " CNnCameraDemo  SetROI!  Nncam_get_Roi success ";
          CUtils::cameraLog(log);
		}


		log = " CNnCameraDemo  SetROI x = " + to_string(static_cast<long long>(x)) + "  y =   " + to_string(static_cast<long long>(y))
			+ "   xsize =  " + to_string(static_cast<long long>(xSize))  + "   ysize = " + to_string(static_cast<long long>(ySize));
        CUtils::cameraLog(log);

        roiX_ = x;
        roiY_ = y;
        m_bROI = true;


        StartCapture();
        ResizeImageBuffer();

		Sleep(2);
		
		unsigned dblExp=0;
		hr = Nncam_get_ExpoTime(g_hcam,&dblExp);
		if(FAILED(hr))
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_get_ExpoTime failed ";
          CUtils::cameraLog(log);
		}
		else
		{
		
		  log = " CNnCameraDemo  SetROI!  Nncam_get_ExpoTime success  dblExp =  " + to_string(static_cast<long long>(dblExp));
          CUtils::cameraLog(log);
		}

		hr = Nncam_put_ExpoTime(g_hcam,  dblExp);
		if(FAILED(hr))
		{
		  log = " CNnCameraDemo  SetROI!  Nncam_put_ExpoTime failed ";
          CUtils::cameraLog(log);
		}
		else
		{
		
		  log = " CNnCameraDemo  SetROI!  Nncam_put_ExpoTime success   dblExp =  " +  to_string(static_cast<long long>(dblExp));
          CUtils::cameraLog(log);
		}

    }

    return DEVICE_OK;
}

/*
* Returns the actual dimensions of the current ROI.
* Required by the MM::Camera API.
*/
int CNnCameraDemo::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{


    x = roiX_;
    y = roiY_;

    xSize = img_.Width();
    ySize = img_.Height();

	string	log = " CNnCameraDemo  GetROI!   x =  "  + to_string(static_cast<long long>(x)) + "  y =  " + to_string(static_cast<long long>(y))
		+ "  xSize =  "  + to_string(static_cast<long long>(xSize))  + "  ySize =  " + to_string(static_cast<long long>(ySize));
    CUtils::cameraLog(log);

    return DEVICE_OK;
}

/*
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int CNnCameraDemo::ClearROI()
{
	string log = " CNnCameraDemo  ClearROI";
    CUtils::cameraLog(log);
    if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

    m_bLiving = false;
    HRESULT hr = Nncam_Stop(g_hcam);      // Stop capture   
    if(FAILED(hr))
	{
		log = " CNnCameraDemo  ClearROI!  Nncam_Stop failed ";
        CUtils::cameraLog(log);
	}
	else
	{
		
		log = " CNnCameraDemo  ClearROI!  Nncam_Stop success ";
        CUtils::cameraLog(log);
	}
	
	ReleaseBuffer();

    roiX_ = 0;
    roiY_ = 0;
    m_bROI = false;

    StartCapture();
    ResizeImageBuffer();

    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Private CNNCam methods
///////////////////////////////////////////////////////////////////////////////

/*
* Sync internal image buffer size to the chosen property values.
*/
int CNnCameraDemo::ResizeImageBuffer()
{
	string	log = " CNnCameraDemo  ResizeImageBuffer! ";
    CUtils::cameraLog(log);
    /*
	if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

    if (NULL == g_pImageData)
        return DEVICE_OUT_OF_MEMORY;

	*/

	int nWidth = 0, nHeight = 0;
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);

	unsigned short ucChannels = TDIBWIDTHBYTES(nWidth * mbiBitCount)/nWidth;


    char buf[MM::MaxStrLength];
    //int ret = GetProperty(MM::g_Keyword_Binning, buf);
    //if (ret != DEVICE_OK)
    //   return ret;
    //binSize_ = atol(buf);

    int ret = GetProperty(MM::g_Keyword_PixelType, buf);
    if (ret != DEVICE_OK)
        return ret;

    std::string pixelType(buf);

    int byteDepth = 0;

    if (pixelType.compare(g_PixelType_8bit) == 0)
    {
        byteDepth = 1;
    }
    else if (pixelType.compare(g_PixelType_16bit) == 0)
    {
        byteDepth = 2;
    }
	else if (pixelType.compare(g_PixelType_24bitRGB) == 0)
    {
        byteDepth = 3;
    }
    else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
    {
        byteDepth = 4;
    }
    else if ( pixelType.compare(g_PixelType_32bit) == 0)
    {
        byteDepth = 4;
    }
    else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
    {
        byteDepth = 8;
    }
		

    log = " CNnCameraDemo  ResizeImageBuffer!   m_header.biWidth =   "  +  to_string(static_cast<long long>(m_header.biWidth))
		+ "  m_header.biHeight =   "  +  to_string(static_cast<long long>(m_header.biHeight))
		+ "   byteDepth = " + to_string(static_cast<long long>(byteDepth))
		+"   nWidth =  " + to_string(static_cast<long long>(nWidth))
		+ "  nHeight =  " + to_string(static_cast<long long>(nHeight))
		+ "   pixelType =  " + pixelType;
    CUtils::cameraLog(log);


	if(m_header.biWidth > 0 && m_header.biHeight > 0)
	{
	  img_.Resize(m_header.biWidth, m_header.biHeight, byteDepth);
	}
	else
	{
	 img_.Resize(nWidth, nHeight, byteDepth);	
	}
    

// #ifdef _WIN64
//     img_.Resize(valWidth.nValue, valHeight.nValue, (m_frame.ucElemBytes * nChnnels));
// #else
//     // We don't use the 16bit data in this app, because of the win32 memory not allowed to create large buffer.
//     img_.Resize(valWidth.nValue, valHeight.nValue, (1/*m_frame.ucElemBytes*/ * nChnnels));
// #endif
//     
    return DEVICE_OK;
}

/*
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CNnCameraDemo::StartSequenceAcquisition(double interval)
{
	string	log = " CNnCameraDemo  StartSequenceAcquisition!   interval = " + to_string(static_cast<long long>(interval));
    CUtils::cameraLog(log);
    return StartSequenceAcquisition(LONG_MAX, interval, false);            
}

/*
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int CNnCameraDemo::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
    
    string log = " CNnCameraDemo  StartSequenceAcquisition  numImages =  " + to_string(static_cast<long long>(numImages)) 
		+ "  interval_ms = "+ to_string(static_cast<long long>(interval_ms))
		+ "  stopOnOverflow =  " + to_string(static_cast<long long>(stopOnOverflow))
		+ " m_bLiving =  " + to_string(static_cast<long long>(m_bLiving));
    CUtils::cameraLog(log);

	snapFlag_ = 3 ;
    if (IsCapturing())
        return DEVICE_CAMERA_BUSY_ACQUIRING;

    if (m_bLiving)
    {
       StopCapture();
    }

    // Switch to standard trigger mode if we are currently in software trigger mode
    int val = 0;
	Nncam_get_Option(g_hcam, NNCAM_OPTION_TRIGGER, &val);
	if (val == 1) 
	{
		returnToSoftwareTriggers_ = true;
		//m_tgrAttr.nTgrMode = TUCCM_SEQUENCE;
		Nncam_put_Option(g_hcam, NNCAM_OPTION_TRIGGER, 0);
	}

    // initialize image buffer
    int nRet = StartCapture();
    if (nRet != DEVICE_OK)
        return nRet;

    int ret = GetCoreCallback()->PrepareForAcq(this);
    if (ret != DEVICE_OK)
        return ret;

    sequenceStartTime_ = GetCurrentMMTime();
    imageCounter_ = 0;
    thd_->Start(numImages,interval_ms);
    stopOnOverflow_ = stopOnOverflow;

	log = " CNnCameraDemo  StartSequenceAcquisition  end  " ;
    CUtils::cameraLog(log);

    return DEVICE_OK;
}

int CNnCameraDemo::IsExposureSequenceable(bool& isSequenceable) const
{
	string	log = " CNnCameraDemo  IsExposureSequenceable!  isSequenceable = " + to_string(static_cast<long long>(isSequenceable));
    CUtils::cameraLog(log);
    isSequenceable = isSequenceable_;
    return DEVICE_OK;
}

int CNnCameraDemo::GetExposureSequenceMaxLength(long& nrEvents) const
{
	string	log = " CNnCameraDemo  GetExposureSequenceMaxLength! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_) 
    {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    nrEvents = sequenceMaxLength_;
    return DEVICE_OK;
}

int CNnCameraDemo::StartExposureSequence()
{

	string	log = " CNnCameraDemo  StartExposureSequence! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = true;
    return DEVICE_OK;
}

int CNnCameraDemo::StopExposureSequence()
{
	string	log = " CNnCameraDemo  StopExposureSequence! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_) 
    {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = false;
    sequenceIndex_ = 0;
    return DEVICE_OK;
}

/*
 * Clears the list of exposures used in sequences
 */
int CNnCameraDemo::ClearExposureSequence()
{

	string	log = " CNnCameraDemo  ClearExposureSequence! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_) 
    {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.clear();
    return DEVICE_OK;
}

/*
 * Adds an exposure to a list of exposures used in sequences
 */
int CNnCameraDemo::AddToExposureSequence(double exposureTime_ms) 
{
	string	log = " CNnCameraDemo  AddToExposureSequence! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_)
    {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.push_back(exposureTime_ms);
    return DEVICE_OK;
}

int CNnCameraDemo::SendExposureSequence() const 
{
	string	log = " CNnCameraDemo  SendExposureSequence! ";
    CUtils::cameraLog(log);
    if (!isSequenceable_) 
    {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    return DEVICE_OK;
}

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int CNnCameraDemo::InsertImage()
{
	string	log = " CNnCameraDemo  InsertImage! ";
    CUtils::cameraLog(log);
    MM::MMTime timeStamp = this->GetCurrentMMTime();
    char label[MM::MaxStrLength];
    this->GetLabel(label);

    // Important:  metadata about the image are generated here:
    Metadata md;
    md.put("Camera", label);
    md.put(MM::g_Keyword_Metadata_StartTime, CDeviceUtils::ConvertToString(sequenceStartTime_.getMsec()));
    md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
    md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString( (long) roiX_)); 
    md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString( (long) roiY_)); 

    imageCounter_++;

//     char buf[MM::MaxStrLength];
//     GetProperty(MM::g_Keyword_Binning, buf);
//     md.put(MM::g_Keyword_Binning, buf);

    char szTemp[256] = {0};
    sprintf(szTemp, "%.3f", m_fCurTemp);
    md.put("Temperature", szTemp); 

    MMThreadGuard g(imgPixelsLock_);

    const unsigned char* pI;
    pI = GetImageBuffer();

    unsigned int w = GetImageWidth();
    unsigned int h = GetImageHeight();
    unsigned int b = GetImageBytesPerPixel();

    int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str());

    if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
    {
        // do not stop on overflow - just reset the buffer
        GetCoreCallback()->ClearImageBuffer(this);
        // don't process this same image again...
        return GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str(), false);
    } else
        return ret;
}

/*
 * called from the thread function before exit 
 */
void CNnCameraDemo::OnThreadExiting() throw()
{
   try
   {
      LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
      GetCoreCallback()?GetCoreCallback()->AcqFinished(this,0):DEVICE_OK;
   }
   catch(...)
   {
      LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
   }
}

int CNnCameraDemo::SetAllowedBinning() 
{
    string log = " CNnCameraDemo  SetAllowedBinning";
    CUtils::cameraLog(log);

    if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

    vector<string> binValues;
	//binValues.push_back("0");
	binValues.push_back("1");
	binValues.push_back("2");
	

	/*
	if (scanMode_ < 3)
		binValues.push_back("4");
	if (scanMode_ < 2)
		binValues.push_back("8");
	if (binSize_ == 8 && scanMode_ == 3) {
		SetProperty(MM::g_Keyword_Binning, "2");
	} else if (binSize_ == 8 && scanMode_ == 2) {
		SetProperty(MM::g_Keyword_Binning, "4");
	} else if (binSize_ == 4 && scanMode_ == 3) {
		SetProperty(MM::g_Keyword_Binning, "2");
	}
	*/

    LogMessage("Setting allowed binning settings", true);
    return SetAllowedValues(MM::g_Keyword_Binning, binValues);
}


int CNnCameraDemo::AllocBuffer()
{
	 if (NULL == g_hcam)
        return DEVICE_NOT_CONNECTED;

	 g_pImageData = (BYTE*)malloc(m_header.biSizeImage);
	 if (NULL == g_pImageData)
	 {
	     string log = " CNnCameraDemo  AllocBuffer failed to malloc";
         CUtils::cameraLog(log);
		 printf("failed to malloc\n");
		  return DEVICE_OUT_OF_MEMORY; 
	 }
	 else
	 {
	     string log = " CNnCameraDemo  AllocBuffer success to malloc";
         CUtils::cameraLog(log);
		 printf("success to malloc\n");
	     return DEVICE_OK;
	 }
}


CNNCamDemoThread::CNNCamDemoThread(CNnCameraDemo* pCam)
   :intervalMs_(default_intervalMS)
   ,numImages_(default_numImages)
   ,imageCounter_(0)
   ,stop_(true)
   ,suspend_(false)
   ,camera_(pCam)
   ,startTime_(0)
   ,actualDuration_(0)
   ,lastFrameTime_(0)
{};

CNNCamDemoThread::~CNNCamDemoThread() {};

void CNNCamDemoThread::Stop() 
{
	string	log = " CNnCameraDemo  CTUCamDemoThread Stop! ";
    CUtils::cameraLog(log);
    MMThreadGuard g(this->stopLock_);
    stop_=true;
}

void CNNCamDemoThread::Start(long numImages, double intervalMs)
{

	string	log = " CNnCameraDemo  CTUCamDemoThread Start! ";
    CUtils::cameraLog(log);
    //OutputDebugString("[CTUCamDemoThread]:Start");
    MMThreadGuard g1(this->stopLock_);
    MMThreadGuard g2(this->suspendLock_);
    numImages_=numImages;
    intervalMs_=intervalMs;
    imageCounter_=0;
    stop_ = false;
    suspend_=false;
    activate();
    actualDuration_ = 0;
    startTime_= camera_->GetCurrentMMTime();
    lastFrameTime_ = 0;
}

bool CNNCamDemoThread::IsStopped()
{
    MMThreadGuard g(this->stopLock_);
    return stop_;
}

void CNNCamDemoThread::Suspend() 
{
    MMThreadGuard g(this->suspendLock_);
    suspend_ = true;
}

bool CNNCamDemoThread::IsSuspended()
{
    MMThreadGuard g(this->suspendLock_);
    return suspend_;
}

void CNNCamDemoThread::Resume()
{
    MMThreadGuard g(this->suspendLock_);
    suspend_ = false;
}

int CNNCamDemoThread::svc(void) throw()
{
    int ret=DEVICE_ERR;
    try 
    {
        do
        {  
            ret = camera_->RunSequenceOnThread(startTime_);

        } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_-1);
        if (IsStopped())
            camera_->LogMessage("SeqAcquisition interrupted by the user\n");
    }catch(...){
        camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
    }
    stop_=true;
    actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
    camera_->OnThreadExiting();
    return ret;
}

/*
 * Do actual capturing
 * Called from inside the thread  
 */
int CNnCameraDemo::RunSequenceOnThread(MM::MMTime startTime)
{
    int ret = DEVICE_ERR;

    // Trigger
    if (triggerDevice_.length() > 0) 
    {
        MM::Device* triggerDev = GetDevice(triggerDevice_.c_str());
        if (triggerDev != 0) 
        {
            LogMessage("trigger requested");
            triggerDev->SetProperty("Trigger","+");
        }
    }

    ret = WaitForFrame(img_);
/*   
    if (!fastImage_)
    {
        GenerateSyntheticImage(img_, GetSequenceExposure());
    }
*/
    ret = InsertImage();

    while (((double) (this->GetCurrentMMTime() - startTime).getMsec() / imageCounter_) < this->GetSequenceExposure())
    {
        CDeviceUtils::SleepMs(1);
    }

    if (ret != DEVICE_OK)
    {
        return ret;
    }

    return ret;
}

int CNnCameraDemo::InitNNCamDemoApi()
{
    
	string log = " CNnCameraDemo  InitTUCamDemoApi";
    CUtils::cameraLog(log);
	g_hcam = Nncam_Open(NULL);
    if (NULL == g_hcam)
    {
        printf("no camera found or open failed\n");

		log = " CNnCameraDemo  no camera found or open failed";
		CUtils::cameraLog(log);
        return DEVICE_NOT_CONNECTED;
    }
	else
	{
	   	log = " CNnCameraDemo  camera  open success";
		CUtils::cameraLog(log);
	  
	}

    return DEVICE_OK;
}

int CNnCameraDemo::UninitNNCamDemoApi()
{   
 
	string log = " CNnCameraDemo  UninitTUCamDemoApi";
    CUtils::cameraLog(log);
	/* cleanup */
    Nncam_Close(g_hcam);                       
	g_hcam =  NULL ;
	log = " CNnCameraDemo  UninitTUCamDemoApi  Nncam_Close  g_hcam";
    CUtils::cameraLog(log);

	/*
	Nncam_Close(g_hcamT);    
	g_hcamT = NULL;

	log = " CNnCameraDemo  UninitTUCamDemoApi  Nncam_Close  g_hcamT";
    CUtils::cameraLog(log);
	*/

	ReleaseBuffer();
    return DEVICE_OK;
}

int CNnCameraDemo::ReleaseBuffer()
{
    string log = " CNnCameraDemo  ReleaseBuffer !";
    CUtils::cameraLog(log);
    /*
	if (NULL == g_hcam)
	{
		log = " CNnCameraDemo  ReleaseBuffer DEVICE_NOT_CONNECTED !";
        CUtils::cameraLog(log);
		return DEVICE_NOT_CONNECTED;
	}
	*/

	log = " CNnCameraDemo  DEVICE_CONNECTED !";
    CUtils::cameraLog(log);
	if (g_pImageData)
	{
		free(g_pImageData);
	    g_pImageData = NULL ;
	}

    log = " CNnCameraDemo  g_pImageData free !";
    CUtils::cameraLog(log);

	/*
	if(g_pImageDataT)
	{
		free(g_pImageDataT);
	    g_pImageDataT = NULL ;
	}
	*/

	log = " CNnCameraDemo  g_pImageDataT free !";
    CUtils::cameraLog(log);

	if(g_pImageData == NULL)
	{
	    log = " CNnCameraDemo  ReleaseBuffer success";
        CUtils::cameraLog(log);
		return DEVICE_OK;
	}
	else
	{
	   log = " CNnCameraDemo  ReleaseBuffer failed";
       CUtils::cameraLog(log);
	   return -1;
	}
    
}


