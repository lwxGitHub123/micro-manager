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

#include "NnCamera2.h"
#include "nncam.h"
#include "VersionNo.h"
#include <cstdio> 
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <list>

#include <stdio.h>
#include <direct.h>
#include <io.h>
#include <time.h>
#include <fstream>
#include <iostream>


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

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif


using namespace std;


list<CNnCamera2*> CNN_CamList;
const char* g_CameraDeviceName = "nn_camera2";

const char* g_ControllerName    = "nn_camera2";

const char* g_Keyword_EPIXUnit     = "EPIX_Unit";
const char* g_Keyword_EPIXMultiUnitMask    = "EPIX Units to Open";
const char* g_BaudRate_key        = "Baud Rate";
const char* g_Baud9600            = "9600";
const char* g_Baud115200          = "115200";


static const char* s_PixelType_8bit = "8bit";
static const char* s_PixelType_16bit = "16bit";
static const char* s_PixelType_32bitRGB = "32bitRGB";

MMThreadLock g_serialLock_[64];


int g_iCameraCount = 0;



string getCwd(){  
    //????????????????  
    string path;  
    path = getcwd(NULL,0);  
    return path;  
}  

//??????????????cameraLog??????
string createDirectory(string directoryPath)
{
	cout << " directoryPath = "  << directoryPath << endl;
	directoryPath = directoryPath + "\\" + "cameraLog" + "\\";
	char * p=(char*)directoryPath.c_str();
    if (ACCESS(p, 0) != 0)
    {
        int ret = MKDIR(p);
        if (ret != 0)
        {
            return directoryPath;
        }
    }
    return directoryPath;
}

void cameraLog(string log)
{
	string path = getCwd();  
    cout << path << endl;  
	string logFile = createDirectory(path);
	ofstream File;
	SYSTEMTIME st = { 0 };  
	GetLocalTime(&st);  //???????????? ????????ms
	string nowStr = to_string(static_cast<long long>(st.wYear)) +"-"+ to_string(static_cast<long long>(st.wMonth)) +"-"+ to_string(static_cast<long long>(st.wDay)) +"  "+ to_string(static_cast<long long>(st.wHour)) + ":"+ to_string(static_cast<long long>(st.wMinute))
		+ ":" +to_string(static_cast<long long>(st.wSecond)) + "." +to_string(static_cast<long long>(st.wMilliseconds));
	File.open(logFile + "\\log"+".txt",ios::out | ios::app);
	File<<nowStr + "     " + log<<endl;
	File.close();



	

	//return logFile ;
}


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
  //AddAvailableDeviceName(g_CameraDeviceName, "PCO generic camera adapter");
  RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "Nncamera2 adapter0706");
  cameraLog(" Nncamera2  InitializeModuleData  ");
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
  //PCO_CamList.remove((CPCOCam*) pDevice);
  cameraLog(" CNnCamera2  DeleteDevice  ");
  delete pDevice;
}

MODULE_API MM::Device* CreateDevice(const char* pszDeviceName)
{
  if(pszDeviceName == 0)
    return 0;

  string strName(pszDeviceName);
  
  cameraLog("CNnCamera2  CreateDevice strName = " + strName  + " g_CameraDeviceName = "  + g_CameraDeviceName );

  if(strName == g_CameraDeviceName)
  {
    CNnCamera2 *cnncam = new CNnCamera2();

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
int CNnCamera2::Shutdown()
{
  

   //is_FreeImageMem (hCam, pcImgMem,  memPid);

   //is_ExitCamera (hCam);

   initialized_ = false;
   cameraLog(" CNnCamera2  Shutdown ");

   return DEVICE_OK;
}


CNnCamera2::CNnCamera2():
	CCameraBase<CNnCamera2>(),
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
	g_hcam(NULL),
	pictime_(0.0)
{

	readoutStartTime_ = GetCurrentMMTime();
	m_iCameraNum = 0;
	m_bDemoMode = FALSE;
	//m_pCamera = NULL;
	//m_pCamera = new CCameraWrapper();


	/*
	CPropertyAction* pAct = new CPropertyAction (this, &CNnCamera2::OnEPIXUnit);
    CreateProperty(g_Keyword_EPIXUnit, "1", MM::Integer, false, pAct, true);
  
    vector<string> EPIXUnits;
    EPIXUnits.push_back("1");
    EPIXUnits.push_back("2");
    EPIXUnits.push_back("3");
    EPIXUnits.push_back("4");
    int nRet = SetAllowedValues(g_Keyword_EPIXUnit, EPIXUnits);

    pAct = new CPropertyAction (this, &CNnCamera2::OnEPIXMultiUnitMask);
    CreateProperty(g_Keyword_EPIXMultiUnitMask, "1", MM::Integer, false, pAct, true);
    nRet = SetAllowedValues(g_Keyword_EPIXMultiUnitMask, EPIXUnits);
	*/

	
     // pixel type
   /*
	CPropertyAction* pAct = new CPropertyAction( this, &CNnCamera2::OnPixelType );
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
	CPropertyAction* pAct = new CPropertyAction(this, &CNnCamera2::OnPort);
	CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

	*/
	//SetProperty(MM::g_Keyword_Port, port_.c_str());

	/*
	AddAllowedValue(g_BaudRate_key, g_Baud115200, (long)115200);
	AddAllowedValue(g_BaudRate_key, g_Baud9600, (long)9600);
	*/

	 WriteLog("nn_generic. Error %x in Init!", 0);
	
	 CPropertyAction* pAct = new CPropertyAction(this, &CNnCamera2::OnDemoMode);
     //CreateProperty("DemoMode", "Off", MM::String, false, pAct, true);
    
	 CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);
	 SetProperty(MM::g_Keyword_Port, port_.c_str());

	 /*
	 AddAllowedValue("DemoMode", "Off", 0);
     AddAllowedValue("DemoMode", "On", 1);
	 */

	 AddAllowedValue(g_BaudRate_key, g_Baud115200, (long)115200);
	 AddAllowedValue(g_BaudRate_key, g_Baud9600, (long)9600);

	 //thd_ = new MySequenceThread(this);

	 cameraLog("  CNnCamera2 ");

	 printf(" ***  ");

}



CNnCamera2::~CNnCamera2()
{
	StopSequenceAcquisition();
	g_iCameraCount = 0;
}



void CNnCamera2::WriteLog(char* message, int nErr)
{
  char szmes[300];

  if(nErr != 0)
    sprintf_s(szmes, sizeof(szmes), "Error %x! %s", nErr, message);
  else
    sprintf_s(szmes, sizeof(szmes), "%s", message);
  LogMessage(szmes);
}

// Camera type
int CNnCamera2::OnCameraType(MM::PropertyBase* pProp, MM::ActionType eAct)
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
    /*
	int iinterface = m_pCamera->m_strCamera.strAPIManager.wInterface;
    if (iinterface > 20)
      iinterface = 0;
	

    m_pCamera->GetCameraNameNType(szname, ilen, &icamtype, &iccdtype, &icamid);
	
    if(m_pCamera->m_iCamClass == 3)
      sprintf_s(sztype, 500, "%s - SN:%0X / Interface: %s", szname, icamid, szinterfaces[iinterface]);
    else
      sprintf_s(sztype, 500, "%s", szname);
	  */
	sprintf_s(sztype, 500, "%s", "USB3.0");
    pProp->Set(sztype);
  }

  
  g_hcam = Nncam_Open(NULL);
  if (NULL == g_hcam)
   {
       printf("no camera found or open failed\n");
	   cameraLog( "CNnCamera2  no camera found or open failed  ");
       return 1;
   }
  

  cameraLog( "CNnCamera2  OnCameraType  ");
  return DEVICE_OK;
}



int CNnCamera2::OnDemoMode(MM::PropertyBase* pProp, MM::ActionType eAct)
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


bool CNnCamera2::SupportsDeviceDetection(void)
{
	return true;
}



MM::DeviceDetectionStatus CNnCamera2::DetectDevice(void)
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

			cameraLog( "CNnCamera2  port_ = " + port_  + " serialnum_ " + serialnum_);

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

int CNnCamera2::OnPort(MM::PropertyBase* pProp, MM::ActionType pAct)
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


/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CNnCamera2::StartSequenceAcquisition(double interval) {

   return StartSequenceAcquisition(LONG_MAX, interval, false);            
}

/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int CNnCamera2::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
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


MySequenceThread::MySequenceThread(CNnCamera2* pCam)
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

int CNnCamera2::StopSequenceAcquisition()                                     
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


string Int_to_String(int n)

{
ostringstream stream;

stream<<n;  //n??int????

return stream.str();

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
int CNnCamera2::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
  

  if (xSize % 2 == 0 || ySize %2 == 0 || x%2 == 0 || y%2 == 0){

    //clear ROI
    ClearROI();
    
    return DEVICE_OK;
  }
  else{

	  string ss =  "CNnCamera2  SetROI :  x == " + Int_to_String(x) + " y =="  + Int_to_String(y) + "  xSize ==  " + Int_to_String(xSize) + " ySize= " + Int_to_String(ySize) ;
	  cameraLog(ss);
	  //Nncam_put_Roi(g_hcam, x, y, xSize, ySize);
      return DEVICE_OK;
    }
    
  }

//}


/**
* Returns the actual dimensions of the current ROI.
* Required by the MM::Camera API.
*/
int CNnCamera2::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
  
 
 if (xSize % 2 == 0 || ySize %2 == 0 || x%2 == 0 || y%2 == 0){

    //clear ROI
    ClearROI();
    
    return DEVICE_OK;
  }
  else{

	  string ss =  "CNnCamera2  GetROI :  x == " + Int_to_String(x) + " y =="  + Int_to_String(y) + "  xSize ==  " + Int_to_String(xSize) + " ySize= " + Int_to_String(ySize) ;
	  cameraLog(ss);
	  //Nncam_get_Roi(g_hcam, &x, &y, &xSize, &ySize);
      return DEVICE_OK;
    }


}


/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int CNnCamera2::ClearROI()
{
  
  INT nRet = 1;
  INT IS_SUCCESS = 1 ;
  if(nRet==IS_SUCCESS){

    ClearImageBuffer(img_);
    SetImageMemory(); 
          

    return DEVICE_OK;
  }
  else{
    LogMessage("could not set ROI",true);
    return ERR_ROI_INVALID;
  } 

}



//allocate and activate image memory correspondig to current ROI, binning and pixel depth
int CNnCamera2::SetImageMemory()
{

  int nRet;

 
  return DEVICE_OK;

}

void CNnCamera2::ClearImageBuffer(ImgBuffer& img)
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
int CNnCamera2::GetBinning() const
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
int CNnCamera2::SetBinning(int binF)
{
  
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}



/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CNnCamera2::GetExposure() const
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
void CNnCamera2::SetExposure(double exp)
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
const unsigned char* CNnCamera2::GetImageBuffer()
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
unsigned CNnCamera2::GetImageWidth() const
{
  
   return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CNnCamera2::GetImageHeight() const
{

   return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CNnCamera2::GetImageBytesPerPixel() const
{

   return img_.Depth();
} 

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CNnCamera2::GetBitDepth() const
{

   return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CNnCamera2::GetImageBufferSize() const
{
 
   return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}


//-----------------------------------------------------------------------------
int CNnCamera2::SnapImage( void )
//-----------------------------------------------------------------------------
{
   int result = DEVICE_ERR;
   
   return result;
}


/**
* Obtains device name.
* Required by the MM::Device API.
*/
void CNnCamera2::GetName(char* name) const
{
   // Return the name used to refer to this device adapter
   CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}

///////////////////////////////////////////////////////////////////////////////
// Function name   : CNnCamera2::Initialize
// Description     : Initialize the camera
// Return type     : bool 

int CNnCamera2::Initialize()
{

   MMThreadGuard g(g_serialLock_[UNITSOPENMAP]);
  // setup NN camera
  // ----------------
	if (initialized_)
      return DEVICE_OK;
	
	// Name
	int nRet = CreateProperty("Name", "nn2 camera", MM::String, true);
	if(nRet != DEVICE_OK)
	return nRet;
   
	// Description
	nRet = CreateProperty("Description", "nn2 generic driver module", MM::String, true);
	if(nRet != DEVICE_OK)
	   return nRet;

	CPropertyAction* pAct;

	// camera type (read-only)
	pAct = new CPropertyAction(this, &CNnCamera2::OnCameraType);
	nRet = CreateProperty("CameraType", "", MM::String, true, pAct);
	if(nRet != DEVICE_OK)
	   return nRet;

	initialized_ = true;

	cameraLog(" CNnCamera2   Initialize");

	return DEVICE_OK;
}


std::string CNnCamera2::DoubleToString(double N)
{
    ostringstream ss("");
    ss << N;
    return ss.str();
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


int CNnCamera2::sendCmd(std::string cmd, std::string& out) {
	sendCmd(cmd);
	GetSerialAnswer(port_.c_str(), "\r", out); //Try returning any extra response from the device.
	return DEVICE_OK;
}


int CNnCamera2::sendCmd(std::string cmd) {
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




