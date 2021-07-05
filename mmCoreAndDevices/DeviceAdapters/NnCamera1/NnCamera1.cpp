///////////////////////////////////////////////////////////////////////////////
// FILE:          NnCamera1.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Implements the NN camera device adaptor using EPIX frame grabber.
//				  Supported cameras: Falcon, Kite, OWL, Osprey and Kingfisher.
//				  DemoCamera.cpp modified by liudongbo for NN camera support
//                
// AUTHOR:        DB @ liudongbo, 6/30/2021
//
// COPYRIGHT:     Raptor Photonics Ltd, (2011-2015)
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
 
//#define MMLINUX32

#include "NnCamera1.h"
#include "nncam.h"
#include "VersionNo.h"
#include <cstdio> 
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <list>

#include <iostream>
#include <sys/stat.h>
#if defined (MMLINUX32) || defined(MMLINUX64)
	#include <sys/time.h>
	#include <stdio.h>
	#include <unistd.h>
	#define sprintf_s snprintf
	void Sleep(double x) {usleep(1000.0*x);};
#endif

#ifdef PLEORA
	const char* g_strVersion = "v1.0.3, 10/21/2019";
#else
	const char* g_strVersion = "v1.14.6, 1/28/2020";
#endif

using namespace std;


list<CNnCamera1*> CNN_CamList;
const char* g_CameraDeviceName = "nn_camera";

const char* g_ControllerName    = "nn_camera";

const char* g_Keyword_EPIXUnit     = "EPIX_Unit";
const char* g_Keyword_EPIXMultiUnitMask    = "EPIX Units to Open";
const char* g_BaudRate_key        = "Baud Rate";
const char* g_Baud9600            = "9600";
const char* g_Baud115200          = "115200";


static const char* s_PixelType_8bit = "8bit";
static const char* s_PixelType_16bit = "16bit";
static const char* s_PixelType_32bitRGB = "32bitRGB";




int g_iCameraCount = 0;


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
  //AddAvailableDeviceName(g_CameraDeviceName, "PCO generic camera adapter");
  RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "Nncamera adapter");
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
  //PCO_CamList.remove((CPCOCam*) pDevice);
  delete pDevice;
}

MODULE_API MM::Device* CreateDevice(const char* pszDeviceName)
{
  if(pszDeviceName == 0)
    return 0;

  string strName(pszDeviceName);

  if(strName == g_CameraDeviceName)
  {
    CNnCamera1 *cnncam = new CNnCamera1();

    CNN_CamList.push_front(cnncam);
    return cnncam;
  }
  return 0;
}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int CNnCamera1::Shutdown()
{
  

   //is_FreeImageMem (hCam, pcImgMem,  memPid);

   //is_ExitCamera (hCam);

   initialized_ = false;
   return DEVICE_OK;
}


CNnCamera1::CNnCamera1():
	CCameraBase<CNnCamera1>(),
	initializedDelay_(false),
	initialized_(false),
	baud_(g_Baud9600),
	readoutUs_(0.0),
    bitDepth_(8),
	sequenceStartTime_(0),
	cameraCCDXSize_(512),
    cameraCCDYSize_(512),
	wavelength_(546),
	numTotalLCs_(2),
	numActiveLCs_(2),
	UNITSOPENMAP(1),
	pictime_(0.0)
{

	readoutStartTime_ = GetCurrentMMTime();
	m_iCameraNum = 0;
	m_bDemoMode = FALSE;
	//m_pCamera = NULL;
	//m_pCamera = new CCameraWrapper();


	/*
	CPropertyAction* pAct = new CPropertyAction (this, &CNnCamera1::OnEPIXUnit);
    CreateProperty(g_Keyword_EPIXUnit, "1", MM::Integer, false, pAct, true);
  
    vector<string> EPIXUnits;
    EPIXUnits.push_back("1");
    EPIXUnits.push_back("2");
    EPIXUnits.push_back("3");
    EPIXUnits.push_back("4");
    int nRet = SetAllowedValues(g_Keyword_EPIXUnit, EPIXUnits);

    pAct = new CPropertyAction (this, &CNnCamera1::OnEPIXMultiUnitMask);
    CreateProperty(g_Keyword_EPIXMultiUnitMask, "1", MM::Integer, false, pAct, true);
    nRet = SetAllowedValues(g_Keyword_EPIXMultiUnitMask, EPIXUnits);
	*/

	
     // pixel type
   /*
	CPropertyAction* pAct = new CPropertyAction( this, &CNnCamera1::OnPixelType );
     int result = CreateStringProperty( MM::g_Keyword_PixelType, s_PixelType_8bit, false, pAct );
     assert( result == DEVICE_OK );
     vector<string> pixelTypeValues;
     pixelTypeValues.push_back( s_PixelType_8bit );
     pixelTypeValues.push_back( s_PixelType_16bit );
     pixelTypeValues.push_back( s_PixelType_32bitRGB );
     //pixelTypeValues.push_back( s_PixelType_64bitRGB );
     result = SetAllowedValues( MM::g_Keyword_PixelType, pixelTypeValues );
    */

		// Port:
	/*
	CPropertyAction* pAct = new CPropertyAction(this, &CNnCamera1::OnPort);
	CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

	*/
	//SetProperty(MM::g_Keyword_Port, port_.c_str());
/*
	pAct = new CPropertyAction(this, &CNnCamera1::OnNumActiveLCs);
	CreateProperty("Number of Active LCs", "2", MM::Integer, false, pAct, true);

	pAct = new CPropertyAction(this, &CNnCamera1::OnNumTotalLCs);
	CreateProperty("Total Number of LCs", "2", MM::Integer, false, pAct, true);
	*/

	/*
	pAct = new CPropertyAction(this, &CNnCamera1::OnNumPalEls);
	CreateProperty("Total Number of Palette Elements", "5", MM::Integer, false, pAct, true);

	pAct = new CPropertyAction(this, &CNnCamera1::OnBaud);
	CreateProperty(g_BaudRate_key, "Undefined", MM::String, false, pAct, true);
	*/

	/*
	AddAllowedValue(g_BaudRate_key, g_Baud115200, (long)115200);
	AddAllowedValue(g_BaudRate_key, g_Baud9600, (long)9600);
	*/

	 WriteLog("pco_generic. Error %x in Init!", 0);
	
	 CPropertyAction* pAct = new CPropertyAction(this, &CNnCamera1::OnDemoMode);
     CreateProperty("DemoMode", "Off", MM::String, false, pAct, true);
     AddAllowedValue("DemoMode", "Off", 0);
     AddAllowedValue("DemoMode", "On", 1);
	 

	 //thd_ = new MySequenceThread(this);

	 printf(" ***  ");

}


CNnCamera1::~CNnCamera1()
{
	StopSequenceAcquisition();
	g_iCameraCount = 0;
}


void CNnCamera1::WriteLog(char* message, int nErr)
{
  char szmes[300];

  if(nErr != 0)
    sprintf_s(szmes, sizeof(szmes), "Error %x! %s", nErr, message);
  else
    sprintf_s(szmes, sizeof(szmes), "%s", message);
  LogMessage(szmes);
}

// Camera type
/*
int CNnCamera1::OnCameraType(MM::PropertyBase* pProp, MM::ActionType eAct)
{
  if(eAct == MM::BeforeGet)
  {
    char szinterfaces[20][40] = {
        "Not specified",
        "FireWire",          // 1            // Firewire interface
        "CL Matrox",         // 2            // Cameralink Matrox Solios / Helios
        "CL Silicon Soft ME3",// 3           // Cameralink Silicon Software Me3
        "CL National Instr.",// 4            // Cameralink National Instruments
        "GigE",              // 5            // Gigabit Ethernet
        "USB",               // 6            // USB 2.0
        "CL Silicon Soft.Me4", // 7          // Cameralink Silicon Software Me4
        "USB3",              // 8            // USB 3.0
        "WLAN",              // 9            // WLan
        "CL Serial Int."    ,// 10           // Cameralink serial
        "CLHS",             // 11           // Cameralink HS Silicon Software Me5
        "Not specified 12",
        "Not specified 13",
        "Not specified 14",
        "Not specified 15",
        "Not specified 16",
        "Not specified 17",
        "Not specified 18",
        "Not specified 19"
    };
    char sztype[500];
    char szname[100];
    int ilen = 100, icamtype = 0, iccdtype = 0, icamid = 0;
    int iinterface = m_pCamera->m_strCamera.strAPIManager.wInterface;
    if (iinterface > 20)
      iinterface = 0;

    m_pCamera->GetCameraNameNType(szname, ilen, &icamtype, &iccdtype, &icamid);
    if(m_pCamera->m_iCamClass == 3)
      sprintf_s(sztype, 500, "%s - SN:%0X / Interface: %s", szname, icamid, szinterfaces[iinterface]);
    else
      sprintf_s(sztype, 500, "%s", szname);
    pProp->Set(sztype);
  }
  return DEVICE_OK;
}
*/


int CNnCamera1::OnDemoMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
  if(eAct == MM::BeforeGet)
  {
    if(m_bDemoMode)
      pProp->Set("On");
    else
      pProp->Set("Off");
  }
  else if(eAct == MM::AfterSet)
  {
    string tmp;
    long demoModeTmp;
    pProp->Get(tmp);
    ((MM::Property *) pProp)->GetData(tmp.c_str(), demoModeTmp);
    m_bDemoMode = (demoModeTmp == 1);
  }
  return DEVICE_OK;
}


bool CNnCamera1::SupportsDeviceDetection(void)
{
	return true;
}



MM::DeviceDetectionStatus CNnCamera1::DetectDevice(void)
{
	
	// all conditions must be satisfied...
	MM::DeviceDetectionStatus result = MM::Misconfigured;

	try
	{
		long baud;
		GetProperty(g_BaudRate_key, baud);

		std::string transformed = port_;
		for (std::string::iterator its = transformed.begin(); its != transformed.end(); ++its)
		{
			*its = (char)tolower(*its);
		}

		if (0 < transformed.length() && 0 != transformed.compare("undefined") && 0 != transformed.compare("unknown"))
		{
			int ret = 0;
			MM::Device* pS;


			// the port property seems correct, so give it a try
			result = MM::CanNotCommunicate;
			// device specific default communication parameters
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_AnswerTimeout, "2000.0");
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_BaudRate, baud_.c_str());
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_DelayBetweenCharsMs, "0.0");
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_Handshaking, "Off");
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_Parity, "None");
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_StopBits, "1");
			GetCoreCallback()->SetDeviceProperty(port_.c_str(), "Verbose", "1");
			pS = GetCoreCallback()->GetDevice(this, port_.c_str());
			pS->Initialize();

			ClearPort(*this, *GetCoreCallback(), port_);
			ret = sendCmd("V?", serialnum_);
			if (ret != DEVICE_OK || serialnum_.length() < 5)
			{
				LogMessageCode(ret, true);
				LogMessage(std::string("NnCamera not found on ") + port_.c_str(), true);
				LogMessage(std::string("NnCamera serial no:") + serialnum_, true);
				ret = 1;
				serialnum_ = "0";
				pS->Shutdown();
			}
			else
			{
				// to succeed must reach here....
				LogMessage(std::string("NnCamera found on ") + port_.c_str(), true);
				LogMessage(std::string("NnCamera serial no:") + serialnum_, true);
				result = MM::CanCommunicate;
				GetCoreCallback()->SetSerialProperties(port_.c_str(),
					"600.0",
					baud_.c_str(),
					"0.0",
					"Off",
					"None",
					"1");
				serialnum_ = "0";
				pS->Initialize();
				ret = sendCmd("R1");
				pS->Shutdown();
			}
		}
	}
	catch (...)
	{
		LogMessage("Exception in DetectDevice!", false);
	}

	return result;

	
}


//////////////// Action Handlers (VariLC) /////////////////

int CNnCamera1::OnPort(MM::PropertyBase* pProp, MM::ActionType pAct)
{
	if (pAct == MM::BeforeGet)
	{
		pProp->Set(port_.c_str());
	}
	else if (pAct == MM::AfterSet)
	{
		if (initialized_)
		{
			pProp->Set(port_.c_str());
			return DEVICE_INVALID_INPUT_PARAM;
		}
		pProp->Get(port_);
	}

	return DEVICE_OK;
}


//-----------------------------------------------------------------------------
/*
void CNnCamera1::RefreshCaptureBufferLayout( void )
//-----------------------------------------------------------------------------
{
   pFI_->getCurrentCaptureBufferLayout( *pIRC_, &pCurrentRequestBufferLayout_ );
   if( pCurrentRequest_ )
   {
      int result = DEVICE_OK;
      LOGGED_MVIMPACT_ACQUIRE_CALL( pCurrentRequest_->unlock, () );
      pCurrentRequest_ = 0;
   }
   const TImageDestinationPixelFormat previousDestinationPixelFormat = pID_->pixelFormat.read();
   switch( pCurrentRequestBufferLayout_->imagePixelFormat.read() )
   {
   case ibpfMono8:
      pID_->pixelFormat.write( idpfMono8 );
      break;
   case ibpfMono10:
   case ibpfMono12:
   case ibpfMono12Packed_V1:
   case ibpfMono12Packed_V2:
   case ibpfMono14:
   case ibpfMono16:
   case ibpfMono32:
      pID_->pixelFormat.write( idpfMono16 );
      break;
   case ibpfBGR888Packed:
   case ibpfRGB888Packed:
      pID_->pixelFormat.write( idpfBGR888Packed );
      break;
   case ibpfRGBx888Packed:
   case ibpfRGBx888Planar:
   case ibpfRGB888Planar:
   case ibpfYUV411_UYYVYY_Packed:
   case ibpfYUV422Packed:
   case ibpfYUV422_UYVYPacked:
   case ibpfYUV422Planar:
   case ibpfYUV444Packed:
   case ibpfYUV444_UYVPacked:
   case ibpfYUV444Planar:
      pID_->pixelFormat.write( idpfRGBx888Packed );
      break;
   // All these could be handled using RGB64 of micro-manager. Unfortunately we deliver RGB48...
   case ibpfBGR101010Packed_V2:
   case ibpfYUV422_10Packed:
   case ibpfYUV422_UYVY_10Packed:
   case ibpfRGB101010Packed:
   case ibpfRGB121212Packed:
   case ibpfRGB141414Packed:
   case ibpfRGB161616Packed:
   case ibpfYUV444_10Packed:
   case ibpfYUV444_UYV_10Packed:
      pID_->pixelFormat.write( idpfRGBx888Packed );
      break;
   case ibpfAuto:
   case ibpfRaw:
      pID_->pixelFormat.write( idpfMono8 );
      break;
      // do NOT add a default here! Whenever the compiler complains it is
      // missing not every format is handled here, which means that at least
      // one has been forgotten and that should be fixed!
   }
   if( previousDestinationPixelFormat != pID_->pixelFormat.read() )
   {
      // in case we did just change the destination pixel format we need to query the
      // current output format once again as the buffer layout might be different now!
      pFI_->getCurrentCaptureBufferLayout( *pIRC_, &pCurrentRequestBufferLayout_ );
   }
}
*/


//-----------------------------------------------------------------------------
int CNnCamera1::OnPixelType( MM::PropertyBase* pProp, MM::ActionType eAct )
//-----------------------------------------------------------------------------
{
   int result = DEVICE_ERR;
   switch( eAct )
   {
   case MM::AfterSet:
      {
         if( IsCapturing() )
         {
            result = DEVICE_CAMERA_BUSY_ACQUIRING;
         }
         else
         {
            string pixelType;
            pProp->Get( pixelType );
            if( //( pixelType.compare( s_PixelType_64bitRGB ) == 0 ) ||
               ( pixelType.compare( s_PixelType_32bitRGB ) == 0 ) ||
               ( pixelType.compare( s_PixelType_16bit ) == 0 ) ||
               ( pixelType.compare( s_PixelType_8bit ) == 0 ) )
            {
               result = DEVICE_OK;
            }
            else
            {
               // on error switch to default pixel type
               pProp->Set( s_PixelType_8bit );
               result = DEVICE_INVALID_PROPERTY_VALUE;
            }
            //RefreshCaptureBufferLayout();
         }
      }
      break;
   case MM::BeforeGet:
      {
         long bytesPerPixel = GetImageBytesPerPixel();
         if( bytesPerPixel == 1 )
         {
            pProp->Set( s_PixelType_8bit );
            result = DEVICE_OK;
         }
         else if( bytesPerPixel == 2 )
         {
            pProp->Set( s_PixelType_16bit );
            result = DEVICE_OK;
         }
         else if( bytesPerPixel == 4 )
         {
            pProp->Set( s_PixelType_32bitRGB );
            result = DEVICE_OK;
         }
         else if( bytesPerPixel == 8 )
         {
            //pProp->Set(s_PixelType_64bitRGB);
            result = DEVICE_INVALID_PROPERTY_VALUE;
         }
         else
         {
            pProp->Set( s_PixelType_8bit );
            result = DEVICE_INVALID_PROPERTY_VALUE;
         }

      }
      break;
   default:
      break;
   }
   return result;
}


int CNnCamera1::OnEPIXMultiUnitMask(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long val = 0;
      pProp->Get(val);

	  if(val!=MULTIUNITMASK)
	  {
		MULTIUNITMASK = (int)val;
	  }
   }
   else if (eAct == MM::BeforeGet)
   {
      pProp->Set((long)MULTIUNITMASK);
   }

   return DEVICE_OK;
}

int CNnCamera1::OnEPIXUnit(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::AfterSet)
   {
      long val = 0;
      pProp->Get(val);

	  if(val!=UNITSOPENMAP)
	  {
		UNITSOPENMAP = (int)val;
	  }
   }
   else if (eAct == MM::BeforeGet)
   {
	   long val = UNITSOPENMAP; 
       pProp->Set(val);
   }

   return DEVICE_OK;
}


/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CNnCamera1::StartSequenceAcquisition(double interval) {

   return StartSequenceAcquisition(LONG_MAX, interval, false);            
}

/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int CNnCamera1::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
   
   if (IsCapturing())
      return DEVICE_CAMERA_BUSY_ACQUIRING;

   int ret = GetCoreCallback()->PrepareForAcq(this);
   if (ret != DEVICE_OK)
      return ret;
   sequenceStartTime_ = GetCurrentMMTime();
   imageCounter_ = 0;
   thd_->Start(numImages,interval_ms);
   stopOnOverflow_ = stopOnOverflow;
   return DEVICE_OK;
}


MySequenceThread::MySequenceThread(CNnCamera1* pCam)
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

MySequenceThread::~MySequenceThread() {};


void MySequenceThread::Stop() {
   MMThreadGuard(this->stopLock_);
   stop_=true;
}

void MySequenceThread::Start(long numImages, double intervalMs)
{
   MMThreadGuard(this->stopLock_);
   MMThreadGuard(this->suspendLock_);
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

bool MySequenceThread::IsStopped(){
   MMThreadGuard(this->stopLock_);
   return stop_;
}

void MySequenceThread::Suspend() {
   MMThreadGuard(this->suspendLock_);
   suspend_ = true;
}

bool MySequenceThread::IsSuspended() {
   MMThreadGuard(this->suspendLock_);
   return suspend_;
}

void MySequenceThread::Resume() {
   MMThreadGuard(this->suspendLock_);
   suspend_ = false;
}

int MySequenceThread::svc(void) throw()
{
   int ret=DEVICE_ERR;
   try 
   {
      do
      {  
         ret=camera_->ThreadRun();
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


/**                                                                       
* Stop and wait for the Sequence thread finished                                   
*/                                                                        
int CNnCamera1::StopSequenceAcquisition()                                     
{
   if (IsCallbackRegistered())
   {
      
   }

   if (!thd_->IsStopped()) {
      thd_->Stop();                                                       
      thd_->wait();                                                       
   }                                                                      
                                                                          
   return DEVICE_OK;                                                      
} 


/**
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but will try as close as possible.
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/
int CNnCamera1::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
  
	/*
  IS_RECT rectAOI;
  IS_RECT rectAOI2;
  IS_SIZE_2D sizeMin;
  IS_SIZE_2D sizeInc;
  */

  if (xSize == 0 && ySize == 0){

    //clear ROI
    ClearROI();
    
    return DEVICE_OK;
  }
  else{

    /*
    //stop acquisition
    is_StopLiveVideo (hCam, IS_FORCE_VIDEO_STOP);
    */  

    printf("IDS_uEye: ROI of %d x %d pixel requested\n", xSize, ySize);
/*
    //read out the current ROI
    is_AOI(hCam, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI2, sizeof(rectAOI2)); 

    //check ROI parameters and define the ROI rectangle
    is_AOI(hCam, IS_AOI_IMAGE_GET_SIZE_INC , (void*)&sizeInc, sizeof(sizeInc)); 
    is_AOI(hCam, IS_AOI_IMAGE_GET_SIZE_MIN , (void*)&sizeMin, sizeof(sizeMin)); 

    //printf("minimal ROI size: %d x %d pixels\n", sizeMin.s32Width, sizeMin.s32Height);
    //printf("ROI increment: x:%d  y:%d pixels\n", sizeInc.s32Width, sizeInc.s32Height);

    rectAOI.s32X = x;
    rectAOI.s32Y = y;

    if (((long)xSize > sizeMin.s32Width) && ((long)xSize <= cameraCCDXSize_)) {
      rectAOI.s32Width= xSize / sizeInc.s32Width * sizeInc.s32Width;
    }
    if (((long)ySize > sizeMin.s32Height) && ((long)ySize <= cameraCCDYSize_)) {
      rectAOI.s32Height= ySize / sizeInc.s32Height * sizeInc.s32Height;
    }
    

    //apply ROI  
    INT nRet = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI)); 

    if(nRet==IS_SUCCESS){


      //update ROI parameters
      roiX_=rectAOI.s32X;
      roiY_=rectAOI.s32Y;      
      roiXSize_=rectAOI.s32Width;
      roiYSize_=rectAOI.s32Height;

      //read out the ROI
      is_AOI(hCam, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI2, sizeof(rectAOI2)); 
      printf("IDS_uEye: ROI of %d x %d pixel obtained\n", rectAOI2.s32Width, rectAOI2.s32Height );

      roiXSizeReal_=rectAOI2.s32Width*binX_;
      roiYSizeReal_=rectAOI2.s32Height*binY_;

 
      //adjust image memory
      is_FreeImageMem (hCam, pcImgMem,  memPid);
      ResizeImageBuffer();
      ClearImageBuffer(img_);
      SetImageMemory();

      
      //update frame rate range
      GetFramerateRange(hCam, &framerateMin_, &framerateMax_);
      SetPropertyLimits("Frame Rate", framerateMin_, framerateMax_);
      //printf("new frame rate range %f %f\n", framerateMin_, framerateMax_);

      //update the exposure range
      GetExposureRange(hCam, &exposureMin_, &exposureMax_, &exposureIncrement_);
      if(exposureMax_>EXPOSURE_MAX) exposureMax_=EXPOSURE_MAX;                  //limit exposure time to keep the interface responsive
      SetPropertyLimits(MM::g_Keyword_Exposure, exposureMin_, exposureMax_);

	  */

      //FIXME: device properties browser window needs to be refreshed here

      return DEVICE_OK;
    }
    /*
    else{
      
      printf("IDS_uEye: ROI of %d x %d pixel requested\n", xSize, ySize);
      LogMessage("IDS_uEye: could not set ROI",true);
      return ERR_ROI_INVALID;
    }
	*/
    
  }

//}


/**
* Returns the actual dimensions of the current ROI.
* Required by the MM::Camera API.
*/
int CNnCamera1::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
  
   x = roiX_;
   y = roiY_;

   xSize = roiXSize_;
   ySize = roiYSize_;

   return DEVICE_OK;
}


/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int CNnCamera1::ClearROI()
{

  /*
  IS_RECT rectAOI;


  rectAOI.s32X     = 0;
  rectAOI.s32Y     = 0;
  rectAOI.s32Width = cameraCCDXSize_/binX_;
  rectAOI.s32Height =cameraCCDYSize_/binY_;
  
  
  //apply ROI  
  INT nRet = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI)); 
  */
   
  INT nRet = 1;
  INT IS_SUCCESS = 1 ;
  if(nRet==IS_SUCCESS){

    
    //update stored ROI parameters
   /*
	roiX_=rectAOI.s32X;
    roiY_=rectAOI.s32Y;      
    roiXSize_=rectAOI.s32Width;
    roiYSize_=rectAOI.s32Height;
    roiXSizeReal_=rectAOI.s32Width*binX_;
    roiYSizeReal_=rectAOI.s32Height*binY_;
	*/


    //adjust image memory
    //is_FreeImageMem (hCam, pcImgMem,  memPid);
    //ResizeImageBuffer();
    ClearImageBuffer(img_);
    SetImageMemory(); 
          

    //update pixel clock range
    //GetPixelClockRange(hCam, &pixelClkMin_, &pixelClkMax_);
   // SetPropertyLimits("Pixel Clock",pixelClkMin_, pixelClkMax_); 
    

    //update frame rate range
   // GetFramerateRange(hCam, &framerateMin_, &framerateMax_);
   // SetPropertyLimits("Frame Rate", framerateMin_, framerateMax_);
     
    
    //update the exposure range
    //GetExposureRange(hCam, &exposureMin_, &exposureMax_, &exposureIncrement_);
   // if(exposureMax_>EXPOSURE_MAX) exposureMax_=EXPOSURE_MAX;                       //limit exposure time to keep the interface responsive
   // SetPropertyLimits(MM::g_Keyword_Exposure, exposureMin_, exposureMax_);  


    return DEVICE_OK;
  }
  else{
    LogMessage("could not set ROI",true);
    return ERR_ROI_INVALID;
  } 

}



//allocate and activate image memory correspondig to current ROI, binning and pixel depth
int CNnCamera1::SetImageMemory()
{

  int nRet;

  //allocate image memory for the current size
  //the size of the ROI already contains binning

  /*
  //  nRet = is_AllocImageMem (hCam, roiXSizeReal_/binX_, roiYSizeReal_/binY_, bitDepthReal_, &pcImgMem, &memPid);
  */


  /*
  //method 1: assign extra memory area which is then copied into the buffer
  nRet = is_AllocImageMem (hCam, roiXSizeReal_/binX_, roiYSizeReal_/binY_, bitDepthReal_, &pcImgMem, &memPid);
  if (nRet != IS_SUCCESS){                          //could not allocate memory
    LogMessage("could not allocate ROI image memory",true);
    return ERR_MEM_ALLOC;
  }
 
  */

  //method 2: directly assign the buffer to the camera
 /*
  pcImgMem=(char*)img_.GetPixelsRW();
  nRet = is_SetAllocatedImageMem (hCam, roiXSizeReal_/binX_, roiYSizeReal_/binY_, 8*img_.Depth(), pcImgMem, &memPid);
  if (nRet != IS_SUCCESS){                          //could not assign image memory
    printf("IDS_uEye: could not assign the image buffer as the image memory, error %d\n", nRet);
  }
  */

  
  //activate the new image memory
  /*
  nRet = is_SetImageMem (hCam, pcImgMem, memPid);
  if (nRet != IS_SUCCESS){                          //could not activate image memory
    printf("IDS_uEye: could not activate image memory, error %d\n", nRet);
  }
  */

  return DEVICE_OK;

}

void CNnCamera1::ClearImageBuffer(ImgBuffer& img)
{
   MMThreadGuard g(imgPixelsLock_);
   if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
      return;
   unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
   memset(pBuf, 0, img.Height()*img.Width()*img.Depth());
}


/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int CNnCamera1::GetBinning() const
{
 
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
int CNnCamera1::SetBinning(int binF)
{
  
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}



/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CNnCamera1::GetExposure() const
{
  
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Exposure, buf);
   if (ret != DEVICE_OK)
      return 0.0;
   return atof(buf);
}

/**
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void CNnCamera1::SetExposure(double exp)
{
   exposureSet_=exp;
   //is_SetExposureTime (hCam, exposureSet_, &exposureCur_);            //deprecated since v. 4.0
   //is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, &exposureSet_, sizeof(exposureSet_));
   //is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, &exposureCur_, sizeof(exposureCur_));

   //SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exposureCur_));

   //GetCoreCallback()->OnExposureChanged(this, exposureCur_);            //FIXME: what does this do?
}


/**
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHeight() and GetImageBytesPerPixel().
* The calling program allso assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* CNnCamera1::GetImageBuffer()
{
  
   MMThreadGuard g(imgPixelsLock_);
   MM::MMTime readoutTime(readoutUs_);
   while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}		
   unsigned char *pB = (unsigned char*)(img_.GetPixels());
   return pB;
}


/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CNnCamera1::GetImageWidth() const
{
  
   return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CNnCamera1::GetImageHeight() const
{

   return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CNnCamera1::GetImageBytesPerPixel() const
{

   return img_.Depth();
} 

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CNnCamera1::GetBitDepth() const
{

   return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CNnCamera1::GetImageBufferSize() const
{
 
   return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}


//-----------------------------------------------------------------------------
int CNnCamera1::SnapImage( void )
//-----------------------------------------------------------------------------
{
   int result = DEVICE_ERR;
   
   return result;
}


/**
* Obtains device name.
* Required by the MM::Device API.
*/
void CNnCamera1::GetName(char* name) const
{
   // Return the name used to refer to this device adapter
   CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}

///////////////////////////////////////////////////////////////////////////////
// Function name   : CNnCamera1::Initialize
// Description     : Initialize the camera
// Return type     : bool 

int CNnCamera1::Initialize()
{
  // setup NN camera
  // ----------------


  	for (long i = 0; i < numTotalNNc_; ++i) {
		retardance_[i] = 0.5;
	}

	for (long i = 0; i < numPalEls_; i++) {
		palEl_[i] = "";
	}

	/*
	HNncam g_hcam = NULL;
	g_hcam = Nncam_Open(NULL);
    if (NULL == g_hcam)
    {
        printf("no camera found or open failed\n");
        return -1;
    }
	*/

	// empty the Rx serial buffer before sending command
/*
	int ret = ClearPort(*this, *GetCoreCallback(), port_);
	if (ret != DEVICE_OK)
		return ret;

	// Name
	ret = CreateProperty(MM::g_Keyword_Name, g_ControllerName, MM::String, true);
	if (DEVICE_OK != ret)
		return ret;

	// Version number
	CPropertyAction* pAct = new CPropertyAction(this, &CNnCamera1::OnSerialNumber);
	ret = CreateProperty("Version Number", "Version Number Not Found", MM::String, true, pAct);
	if (ret != DEVICE_OK)
		return ret;

	// Active LC number
	pAct = new CPropertyAction(this, &CNnCamera1::OnNumActiveLCs);
	ret = CreateProperty("Active LCs", "0", MM::Integer, true, pAct);
	if (ret != DEVICE_OK)
		return ret;

	// Total LC number
	pAct = new CPropertyAction(this, &CNnCamera1::OnNumTotalLCs);
	ret = CreateProperty("Total LCs", "0", MM::Integer, true, pAct);
	if (ret != DEVICE_OK)
		return ret;

	pAct = new CPropertyAction(this, &CNnCamera1::OnBriefMode);
	ret = CreateProperty("Mode; 1=Brief; 0=Standard", "", MM::String, true, pAct);
	if (ret != DEVICE_OK)
		return ret;

	//Set VariLC to Standard mode
	briefModeQ_ = false;
	ret = sendCmd("B0",getFromVariLC_);
	if (ret != DEVICE_OK)
		return ret;

	// Wavelength
	std::string ans;
	ret = sendCmd("V?", ans);	//The serial number response also contains the tuning range of the device
	std::vector<double> nums = getNumbersFromMessage(ans, briefModeQ_);	//This will be in the format (revision level, shortest wavelength, longest wavelength, serial number).
	if (ret != DEVICE_OK)
		return ret;
	pAct = new CPropertyAction(this, &CNnCamera1::OnWavelength);
	ret = CreateProperty("Wavelength", DoubleToString(wavelength_).c_str(), MM::Float, false, pAct);
	if (ret != DEVICE_OK)
		return ret;
	SetPropertyLimits("Wavelength", nums.at(1), nums.at(2));

	// Delay
	pAct = new CPropertyAction(this, &CNnCamera1::OnDelay);
	ret = CreateProperty("Device Delay (ms.)", "200.0", MM::Float, false, pAct);
	if (ret != DEVICE_OK)
		return ret;
	SetPropertyLimits("Device Delay (ms.)", 0.0, 200.0);

	CPropertyActionEx *pActX = 0;
	//	 create an extended (i.e. array) properties 0 through 1

	for (long i = 0; i < numActiveLCs_; ++i) {
		ostringstream s;
		s << "Retardance LC-" << char(65 + i);
		pActX = new CPropertyActionEx(this, &CNnCamera1::OnRetardance, i);
		CreateProperty(s.str().c_str(), "0.5", MM::Float, false, pActX);
		SetPropertyLimits(s.str().c_str(), 0.0001, 3);
	}

	// Absolute Retardance controls -- after Voltage controls
	for (long i = 0; i < numActiveLCs_; ++i) {
		ostringstream s;
		s << "Retardance LC-" << char(65 + i) << " [in nm.]";
		pActX = new CPropertyActionEx(this, &CNnCamera1::OnAbsRetardance, i);
		CreateProperty(s.str().c_str(), "100", MM::Float, true, pActX);
	}
	for (long i = 0; i < numPalEls_; ++i) {
		ostringstream s;
		std::string number;

		std::stringstream strstream;
		strstream << i;
		strstream >> number;
		if (i < 10) {
			number = "0" + number;
		}
		s << "Pal. elem. " << number << "; enter 0 to define; 1 to activate";
		pActX = new CPropertyActionEx(this, &CNnCamera1::OnPalEl, i);
		CreateProperty(s.str().c_str(), "", MM::String, false, pActX);
	}
	pAct = new CPropertyAction(this, &CNnCamera1::OnSendToVariLC);
	ret = CreateProperty("String send to VariLC", "", MM::String, false, pAct);
	if (ret != DEVICE_OK) {
		return ret;
	}
	pAct = new CPropertyAction(this, &CNnCamera1::OnGetFromVariLC);
	ret = CreateProperty("String from VariLC", "", MM::String, true, pAct);
	if (ret != DEVICE_OK) {
		return ret;
	}
	SetErrorText(99, "Device set busy for ");

	*/


	return DEVICE_OK;
}



int CNnCamera1::OnGetFromVariLC(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	//   GetSerialAnswer (port_.c_str(), "\r", getFromVariLC_);
      pProp->Set(getFromVariLC_.c_str());
   }
   return DEVICE_OK;
}


 int CNnCamera1::OnDelay(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
	 double delay = GetDelayMs();
	 initializedDelay_ = true;
	 pProp->Set(delay);
    }
   else if (eAct == MM::AfterSet)
   {
	  double delayT;
	  pProp->Get(delayT);
	  if (initializedDelay_) {
		SetDelayMs(delayT);
	  }
	  delay = delayT*1000;
   }
   return DEVICE_OK;
}

std::string CNnCamera1::DoubleToString(double N)
{
    ostringstream ss("");
    ss << N;
    return ss.str();
}

int CNnCamera1::OnSerialNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
 {
	 if (eAct == MM::BeforeGet)
	 {
		 int ret = sendCmd("V?", serialnum_);
		 if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
		 pProp->Set(serialnum_.c_str());
	 }
	 return DEVICE_OK;
 }


int CNnCamera1::OnAbsRetardance(MM::PropertyBase* pProp, MM::ActionType eAct, long index)
{	       
   if (eAct == MM::BeforeGet)
   {	 
     if (index+1 > 0) {	    
	   pProp->Set(retardance_[index]*wavelength_);
	 } else {
		 return DEVICE_INVALID_PROPERTY_VALUE;
	 }
   }
   else if (eAct == MM::AfterSet)
   {
	 
   }
   return DEVICE_OK;
}


// General utility function:
int ClearPort(MM::Device& device, MM::Core& core, std::string port)
{
	// Clear contents of serial port 
	const int bufSize = 2048;
	unsigned char clear[bufSize];
	unsigned long read = bufSize;
	int ret;
	while ((int)read == bufSize)
	{
		ret = core.ReadFromSerial(&device, port.c_str(), clear, bufSize, read);
		if (ret != DEVICE_OK)
			return ret;
	}
	return DEVICE_OK;
}



std::vector<double> CNnCamera1::getNumbersFromMessage(std::string variLCmessage, bool briefMode) {
   std::istringstream variStream(variLCmessage);
   std::string prefix;
   double val;
   std::vector<double> values;

   if (!briefMode){
      variStream >> prefix;
	}
    for (;;) {
       variStream >> val;
       if (! variStream.fail()) {
          values.push_back(val);
       } else {
           break;
       }
	 }

	return values;
}

int CNnCamera1::sendCmd(std::string cmd, std::string& out) {
	sendCmd(cmd);
	GetSerialAnswer(port_.c_str(), "\r", out); //Try returning any extra response from the device.
	return DEVICE_OK;
}


int CNnCamera1::sendCmd(std::string cmd) {
	int ret = SendSerialCommand(port_.c_str(), cmd.c_str(), "\r");
	if (ret != DEVICE_OK) {
		return DEVICE_SERIAL_COMMAND_FAILED;
	}
	std::string response;
	GetSerialAnswer(port_.c_str(), "\r", response);	//Read back the response and make sure it matches what we sent. If not there is an issue with communication.
	if (response != cmd) {
		SetErrorText(99, "The VariLC did not respond.");
		return 99;
	}
	return DEVICE_OK;
}


int CNnCamera1::OnRetardance(MM::PropertyBase* pProp, MM::ActionType eAct, long index)
{
	if (eAct == MM::BeforeGet)
	{
		std::string ans;
		int ret = sendCmd("L?", ans);
		if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
		vector<double> numbers = getNumbersFromMessage(ans, briefModeQ_);
		if (index < (int)numbers.size()) {
			retardance_[index] = numbers[index];
			pProp->Set(retardance_[index]);

			ostringstream s;
			s << "Retardance LC-" << char(65 + index) << " [in nm.]";
			std::string s2 = DoubleToString(retardance_[index] * wavelength_);
			SetProperty(s.str().c_str(), s2.c_str());

		}
		else {
			return DEVICE_INVALID_PROPERTY_VALUE;
		}
	}
	else if (eAct == MM::AfterSet)
	{
		double retardance;

		// read value from property
		pProp->Get(retardance);
		// write retardance out to device....
		ostringstream cmd;
		cmd.precision(4);
		cmd << "L";
		for (int i = 0; i < numTotalLCs_; i++) {
			if (i == index) {
				cmd << " " << retardance;
			}
			else {
				cmd << " " << retardance_[i];
			}
		}
		int ret = sendCmd(cmd.str());
		if (ret != DEVICE_OK)
			return DEVICE_SERIAL_COMMAND_FAILED;
		retardance_[index] = retardance;

		changedTime_ = GetCurrentMMTime();
	}
	return DEVICE_OK;
}

int CNnCamera1::OnNumActiveLCs(MM::PropertyBase* pProp, MM::ActionType eAct)
 {
	 if (eAct == MM::BeforeGet)
	 {
		 pProp->Set(numActiveLCs_);
	 }
	 else if (eAct == MM::AfterSet)
	 {
		 pProp->Get(numActiveLCs_);
	 }
	 return DEVICE_OK;
 }


int CNnCamera1::OnNumTotalLCs(MM::PropertyBase* pProp, MM::ActionType eAct)
 {
	 if (eAct == MM::BeforeGet)
	 {
		 pProp->Set(numTotalLCs_);
	 }
	 else if (eAct == MM::AfterSet)
	 {
		 pProp->Get(numTotalLCs_);
	 }
	 return DEVICE_OK;
 }

int CNnCamera1::OnBriefMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	if (eAct == MM::BeforeGet) {
		std::string ans;
		int ret = sendCmd("B?", ans);
		if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
		if (ans == "1") {
			briefModeQ_ = true;
		}
		else {
			briefModeQ_ = false;
		}
		if (briefModeQ_) {
			pProp->Set(" 1");
		}
		else {
			pProp->Set(" 0");
		}
	}
	else if (eAct == MM::AfterSet)
	{

	}
	return DEVICE_OK;
}


int CNnCamera1::OnWavelength(MM::PropertyBase* pProp, MM::ActionType eAct)
 {
	 if (eAct == MM::BeforeGet)
	 {
		 std::string ans;
		 int ret = sendCmd("W?", ans);
		 if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
		 vector<double> numbers = getNumbersFromMessage(ans, briefModeQ_);
		 if (numbers.size() == 0) { //The device must have returned "W*" meaning that an invalid wavelength was sent
			 SetErrorText(99, "The Varispec device was commanded to tune to an out of range wavelength.");
			 return 99;
		 }
		 pProp->Set(numbers[0]);
	 }
	 else if (eAct == MM::AfterSet)
	 {
		 double wavelength;
		 // read value from property
		 pProp->Get(wavelength);
		 // write wavelength out to device....
		 ostringstream cmd;
		 cmd.precision(5);
		 cmd << "W " << wavelength;
		 int ret = sendCmd(cmd.str().c_str());
		 if (ret != DEVICE_OK)
			 return DEVICE_SERIAL_COMMAND_FAILED;
		 wavelength_ = wavelength;
	 }
	 return DEVICE_OK;
 }

int CNnCamera1::OnPalEl(MM::PropertyBase* pProp, MM::ActionType eAct, long index)
{
	 if (eAct == MM::BeforeGet)
	 {
		 std::string ans;
		 int ret = sendCmd("D?", ans);
		 if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
		 vector<double> numbers = getNumbersFromMessage(ans, briefModeQ_);
		 int elemNr = (int)numbers[0];
		 if (elemNr == 0) {
			 for (int i = 0; i < numPalEls_ + 1; i++) {
				 palEl_[i] = "";
			 }
		 }
		 else {
			 for (int i = 1; i <= elemNr; i++) {  //i is one based
				 ret = GetSerialAnswer(port_.c_str(), "\r", ans);
				 if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
				 if ((i - 1) == index) {
					 numbers = getNumbersFromMessage(ans, briefModeQ_);
					 ostringstream palElStr;
					 palElStr.precision(4);
					 palElStr << "  " << (numbers[0]);
					 for (long i = 0; i < numActiveLCs_; ++i) {
						 palElStr << "  " << (numbers[i + 1]);
					 }
					 palEl_[index] = palElStr.str();
				 }
			 }
		 }
		 pProp->Set(palEl_[index].c_str());
	 }
   else if (eAct == MM::AfterSet)
   {
	   long setPalEl = 0;
	   changedTime_ = GetCurrentMMTime();  //enter in each function that sets the LCs
	   ostringstream cmd;
	   cmd.precision(0);
	   pProp->Get(setPalEl);
	   if (setPalEl == 0) {
		   cmd << "D " << index;
		   int ret = sendCmd(cmd.str());
		   if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;

	   }
	   if (setPalEl == 1) {
		   cmd << "P " << index;
		   int ret = sendCmd(cmd.str());
		   if (ret != DEVICE_OK)return DEVICE_SERIAL_COMMAND_FAILED;
	   }
	 }
   return DEVICE_OK;
}


int CNnCamera1::OnSendToVariLC(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	 if (eAct == MM::AfterSet) {
		 // read value from property
		 pProp->Get(sendToVariLC_);
		 return sendCmd(sendToVariLC_, getFromVariLC_);
	 }
	 return DEVICE_OK;
}



