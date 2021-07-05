///////////////////////////////////////////////////////////////////////////////
// FILE:          MicroManager.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   pco camera module
//                
// AUTHOR:        Franz Reitner, Franz.Reitner@pco.de, 11/01/2010
// COPYRIGHT:     PCO AG, Kelheim, 2010
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
//


/*#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif*/

#define PCO_ERRT_H_CREATE_OBJECT

#include "..\..\MMDevice/ModuleInterface.h"
#include "PCO_Camera2.h"
#include "VersionNo.h"
#include "nncam.h"
//#pragma warning(disable : 4996) // disable warning for deperecated CRT functions on Windows 

/*
#if defined _WIN64
#pragma comment(lib, ".\\lib\\pco_generic\\pco_kamlib64.lib")
#else
#pragma comment(lib, ".\\lib\\pco_generic\\pco_kamlib.lib")
#endif
*/

#include <string>
#include <sstream>
#include <list>

using namespace std;

list<CPCO_Camera2*> PCO_CamList;
const char* g_CameraDeviceName = "pco1_camera";

const char* g_PixelType_8bit = "8bit";
const char* g_PixelType_16bit = "16bit";
const char* g_PixelType_RGB32bit = "RGB 32bit";

const char* g_TimeStamp_No = "No stamp";
const char* g_TimeStamp_B = "Binary";
const char* g_TimeStamp_BA = "Binary + ASCII";

int g_iCameraCount = 0;
int g_iSC2Count = 0;
int g_iSenCount = 0;
int g_iPFCount = 0;


HNncam g_hcam = NULL;
void* g_pImageData = NULL;
unsigned g_total = 0;


/* DEVICE_INTERFACE_VERSION is defined in MMDevice.h under MMDevice folder
*/

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
  //AddAvailableDeviceName(g_CameraDeviceName, "PCO1 generic camera adapter");
  RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "PCO1 generic camera adapter");
}

MODULE_API MM::Device* CreateDevice(const char* pszDeviceName)
{
  if(pszDeviceName == 0)
    return 0;

  /*
  CPCO_Camera2 *pcam = new CPCO_Camera2();
  return pcam;
  */

  string strName(pszDeviceName);

  if(strName == g_CameraDeviceName)
  {
    CPCO_Camera2 *pcam = new CPCO_Camera2();

    PCO_CamList.push_front(pcam);
    return pcam;
  }
  return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
  PCO_CamList.remove((CPCO_Camera2*) pDevice);
  delete pDevice;
}


CPCO_Camera2::CPCO_Camera2():
CCameraBase<CPCO_Camera2>(),
m_bInitialized(false),
pictime_(0.0)
{

	
	m_bDemoMode = FALSE;


   // DemoMode (pre-initialization property)
   CPropertyAction* pAct = new CPropertyAction(this, &CPCO_Camera2::OnDemoMode);
   CreateProperty("DemoMode", "Off", MM::String, false, pAct, true);
   AddAllowedValue("DemoMode", "Off", 0);
   AddAllowedValue("DemoMode", "On", 1);



}


CPCO_Camera2::~CPCO_Camera2()
{
  
  if(m_bInitialized)
  {
    Shutdown();
  }


}

void CPCO_Camera2::GetName(char* name) const
{
  // Return the name used to referr to this device adapte
  CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}


// Camera type
int CPCO_Camera2::OnCameraType(MM::PropertyBase* pProp, MM::ActionType eAct)
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
	sprintf_s(sztype, 500, "%s", "USB3");
    pProp->Set(sztype);
  }
  return DEVICE_OK;
}


int CPCO_Camera2::OnDemoMode(MM::PropertyBase* pProp, MM::ActionType eAct)
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


int CPCO_Camera2::InsertImage()
{
  const unsigned char* img;
  MM::Core *pcore = GetCoreCallback();
  int ret = DEVICE_OK;
  if(pcore != NULL)
  {
    int icurrent;

    for(int j = 0; j < 4; j++)
    {
      icurrent = m_iLastBufferUsed[j];
      if(icurrent < 0)
        break;
      m_iNextBuffer = icurrent + 1;
      if(m_iNextBuffer > 3)
        m_iNextBuffer = 0;
      img = GetBuffer(icurrent);
      if(img == 0)
        return ERR_TIMEOUT;

      m_iNumImagesInserted++;
      ret = pcore->InsertImage(this, img, m_iWidth, m_iHeight, m_iBytesPerPixel);
      if(!m_bStopOnOverflow && ret == DEVICE_BUFFER_OVERFLOW)
      {
        // do not stop on overflow - just reset the buffer
        pcore->ClearImageBuffer(this);
        return pcore->InsertImage(this, img, m_iWidth, m_iHeight, m_iBytesPerPixel);
      }
    }
  }
  else                                 // Else path for Unit-Tester
  {
    int icurrent;
    for(int j = 0; j < 4; j++)
    {
      icurrent = m_iLastBufferUsed[j];
      if(icurrent < 0)
        break;
      m_iNextBuffer = icurrent + 1;
      if(m_iNextBuffer > 3)
        m_iNextBuffer = 0;
      img = GetBuffer(icurrent);
      memcpy((void*) img_.GetPixels(), (void*) img, img_.Height() * img_.Width() * img_.Depth());
    }
  }
  return ret;
}


int CPCO_Camera2::ResizeImageBuffer()
{
  // get image size
  int nWidth = 1, nHeight = 1;
  int as;

  int nErr = 0;
  if(m_bDemoMode)
  {
    nWidth = 1280;
    nHeight = 1024;
  }
  else
  {
    
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);
    if(FAILED(hr))
    {
      return nErr;
    }
    m_iWidth = nWidth;
    m_iHeight = nHeight;
  }

  if(!(pixelDepth_ == 1 || pixelDepth_ == 2 || pixelDepth_ == 4))
    return -1;
  img_.Resize(nWidth, nHeight, pixelDepth_);
  /*
  SetSizes(nWidth, nHeight, pixelDepth_);
  if(img_.Depth() == 1)
  {
    m_pCamera->SetConvertBWCol(TRUE, FALSE);
    m_pCamera->SetViewMode(TRUE, FALSE, FALSE, FALSE, 1.0, FALSE);//SetFlip(TRUE);  img_.Resize(nWidth, nHeight, pixelDepth_);
  }
  if(img_.Depth() == 4)
  {
    m_pCamera->SetConvertBWCol(FALSE, TRUE);
    m_pCamera->SetViewMode(TRUE, FALSE, FALSE, FALSE, 1.0, FALSE);//SetFlip(TRUE);  img_.Resize(nWidth, nHeight, pixelDepth_);
  }
  */
  return DEVICE_OK;
}

int CPCO_Camera2::SetupCamera(bool bStopRecording, bool bSizeChanged)
{
  unsigned int uiMode;
  int nErr = 0;
  int istopresult;
  int iOffsPxr;
  bool bwasrecording = m_bRecording;

  if(m_bDemoMode)
  {
    nErr = ResizeImageBuffer();
    if(nErr != 0)
      return nErr;
    return DEVICE_OK;
  }
  if(bStopRecording || m_bSoftwareTriggered)
  {
    if(WaitForSingleObject(mxMutex, 800) == WAIT_TIMEOUT)// Try to lock acquisition thread
    {
      return DEVICE_CAMERA_BUSY_ACQUIRING;
    }
    if(m_bRecording)
      //nErr = m_pCamera->StopCam(&istopresult);
	  Nncam_Stop(g_hcam);
    m_bRecording = false;
    if(nErr != 0)
    {
      ReleaseMutex(mxMutex);
      return nErr;
    }
  }

  iOffsPxr = m_iOffset;

  m_nTimesLen = MM_PCO_GENERIC_MAX_STRLEN;
  if((m_nCameraType == 0x1300) || (m_nCameraType == 0x1310))
    iOffsPxr = m_iPixelRate;
  //if(m_pCamera->m_iCamClass == 3)
  /*
  {
    DWORD dwresult = 0;
    m_pCamera->SetCameraStruct((PCO_Camera*) &m_pCamera->m_strCamera.wSize, &dwresult);

    std:string szsetname = "";
    m_pCamera->WriteSettingsToRegistryMM(0, szsetname, m_pCamera->m_strCamera);
  }
  */
  /*else
  {
    nErr = m_pCamera->testcoc(&m_nMode, &m_nTrig, &m_nRoiXMin, &m_nRoiXMax, &m_nRoiYMin, &m_nRoiYMax,
    &m_nHBin, &m_nVBin, m_pszTimes, &m_nTimesLen, &m_iGain, &iOffsPxr, &m_uiFlags);
    if ((nErr != 0) && (nErr != 103))
      return nErr;

    nErr = m_pCamera->setcoc(m_nMode, m_nTrig, m_nRoiXMin, m_nRoiXMax, m_nRoiYMin, m_nRoiYMax,
    m_nHBin, m_nVBin, m_pszTimes, m_iGain, iOffsPxr, m_uiFlags);
    if (nErr != 0)
      return nErr;
  }*/

  m_bSettingsChanged = TRUE;
  if(bStopRecording || m_bSoftwareTriggered)
  {
    if(bSizeChanged)
    {
      nErr = ResizeImageBuffer();
      if(nErr != 0)
      {
        ReleaseMutex(mxMutex);
        return nErr;
      }
    }
    uiMode = 0x10000 + 0x0010;//Avoid adding buffers, Preview, Single
   /*
	nErr = m_pCamera->PreStartCam(uiMode, 0, 0, 0);            // schaltet automatisch auf internen Trigger
    if(nErr != 0)
    {
      ReleaseMutex(mxMutex);
      return nErr;
    }
	
    if(bwasrecording)
    {
      for(int j = 0; j < 4; j++)
        m_iLastBufferUsed[j] = -1;
      m_iNextBuffer = 0;
      nErr = m_pCamera->StartCam();
      m_bRecording = true;
    }
	*/

    ReleaseMutex(mxMutex);
  }
  return nErr;

}



int CPCO_Camera2::ClearROI()
{
  int nErr = 0;

  if(m_bDemoMode)
    return DEVICE_OK;

  m_nRoiXMin = 1;
  m_nRoiYMin = 1;
  m_nRoiXMax = roiXMaxFull_;
  m_nRoiYMax = roiYMaxFull_;


  nErr = SetupCamera(true, true);

  if(nErr != 0)
    return nErr;

  // Liisa: read the current ROI to the variables to be used in SnapImage
  // Although the values set by SET_COC are correct here, it goes wrong somewhere later
  // and in SnapImage the old ROI is used
  if(m_bSettingsChanged)
  {   
    m_bSettingsChanged = FALSE;
  }
  // end Liisa

  if(nErr != 0)
    return nErr;

  nErr = ResizeImageBuffer();

  if(nErr != 0)
    return nErr;

  return DEVICE_OK;
}

int CPCO_Camera2::SetROI(unsigned uX, unsigned uY, unsigned uXSize, unsigned uYSize)
{
  int nErr = 0;

  if(m_bDemoMode)
    return DEVICE_OK;

  Nncam_put_Roi(g_hcam,uX,uY,uXSize,uYSize);

  return DEVICE_OK;
}

int CPCO_Camera2::GetROI(unsigned& uX, unsigned& uY, unsigned& uXSize, unsigned& uYSize)
{
  int nErr = 0;
  if(m_bDemoMode)
  {
    uXSize = uX = 1280;
    uYSize = uY = 1024;

    return DEVICE_OK;
  }
  
  Nncam_get_Roi(g_hcam,&uX,&uY,&uXSize,&uYSize);

  return DEVICE_OK;
}

void CPCO_Camera2::SetExposure(double dExp)
{
  SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(dExp));
}

int CPCO_Camera2::GetBinning() const
{
  return m_nHBin;
}

int CPCO_Camera2::SetBinning(int binSize)
{
  ostringstream os;
  os << binSize;
  return SetProperty(MM::g_Keyword_Binning, os.str().c_str());
}

unsigned CPCO_Camera2::GetBitDepth() const
{
  if(img_.Depth() == 1)
    return 8;
  else if(img_.Depth() == 2)
  {
    return Nncam_get_MaxBitDepth(g_hcam);
  }
  else if(img_.Depth() == 4)
  {
    return 8;
  }
  else
  {
    return 0; // should not happen
  }
}

unsigned  CPCO_Camera2::GetImageBytesPerPixel() const
{
  /*  if(img_.Depth() == 4)// Faulty setting inside MM
  return 1;
  else*/
  return img_.Depth();
}

const unsigned char* CPCO_Camera2::GetBuffer(int ibufnum)
{
	if(ibufnum < 0)
        return NULL;

	return (unsigned char*)g_pImageData ;
}

const unsigned char* CPCO_Camera2::GetImageBuffer()
{
  return (unsigned char*) GetBuffer(m_iLastBufferUsed[0]);
}

static void __stdcall EventCallback(unsigned nEvent, void* pCallbackCtx)
{
    if (NNCAM_EVENT_IMAGE == nEvent)
    {
        NncamFrameInfoV2 info = { 0 };
        HRESULT hr = Nncam_PullImageV2(g_hcam, g_pImageData, 24, &info);
        if (FAILED(hr))
            printf("failed to pull image, hr = %08x\n", hr);
        else
        {
            /* After we get the image data, we can do anything for the data we want to do */
            printf("pull image ok, total = %u, res = %u x %u\n", ++g_total, info.width, info.height);
			printf("  g_pImageData  ==  ");
			/*
			wchar_t strPath[MAX_PATH];
		    swprintf(strPath, L"%04u.jpg", m_nSnapFile++);
			SaveImageByWIC(strPath, pSnapData, &header);
			*/
        }
    }
    else
    {
        printf("event callback: %d\n", nEvent);
    }
}


///////////////////////////////////////////////////////////////////////////////
// Function name   : CPCOCam::SnapImage
// Description     : Acquires a single frame and stores it in the internal
//                   buffer
// Return type     : bool 

int CPCO_Camera2::SnapImage()
{
    int nErr = 0;

    if(m_bDemoMode)
      return DEVICE_OK;

    g_hcam = Nncam_Open(NULL);
    if (NULL == g_hcam)
    {
        printf("no camera found or open failed\n");
        return -1;
    }
    
    int nWidth = 0, nHeight = 0;
    HRESULT hr = Nncam_get_Size(g_hcam, &nWidth, &nHeight);
    if (FAILED(hr))
        printf("failed to get size, hr = %08x\n", hr);
    else
    {
        g_pImageData = malloc(TDIBWIDTHBYTES(24 * nWidth) * nHeight);
        if (NULL == g_pImageData)
            printf("failed to malloc\n");
        else
        {
            //hr = Nncam_StartPullModeWithCallback(g_hcam, EventCallback, NULL);
			hr = Nncam_Snap(g_hcam,0);
            if (FAILED(hr))
                printf("failed to Snap camera, hr = %08x\n", hr);
            else
            {
                printf("press any key to exit\n");
                //getc(stdin);
            }
        }
    }


    return DEVICE_OK;
}






///////////////////////////////////////////////////////////////////////////////
// Function name   : CPCO_Camera2::Initialize
// Description     : Initialize the camera
// Return type     : bool 

int CPCO_Camera2::Initialize()
{

  // Name
  int nRet = CreateProperty("Name", "pco1 camera", MM::String, true);
  if(nRet != DEVICE_OK)
    return nRet;
   
    // Description
  nRet = CreateProperty("Description", "pco1 generic driver module", MM::String, true);
  if(nRet != DEVICE_OK)
    return nRet;

  CPropertyAction* pAct;

  // camera type (read-only)
  pAct = new CPropertyAction(this, &CPCO_Camera2::OnCameraType);
  nRet = CreateProperty("CameraType", "", MM::String, true, pAct);
  if(nRet != DEVICE_OK)
    return nRet;
  UpdateProperty("CameraType");


  return DEVICE_OK ;


}

///////////////////////////////////////////////////////////////////////////////
// Function name   : CPCOCam::Shutdown
// Description     : Deactivate the camera, reverse the initialization process
// Return type     : bool 

int CPCO_Camera2::Shutdown()
{
  int istopresult = 0;

  /*
  if(m_pCamera != NULL)
  {
    if(!m_bDemoMode)
      m_pCamera->StopCam(&istopresult);
    m_bRecording = false;

    m_pCamera->CloseCam();
  }
  */
  Nncam_Close(g_hcam);
  if (g_pImageData)
     free(g_pImageData);


  //EnableConvert(FALSE);
  return DEVICE_OK;
}




