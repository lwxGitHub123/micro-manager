///////////////////////////////////////////////////////////////////////////////
// FILE:       DiskoveryCommander.cpp
// PROJECT:    MicroManager
// SUBSYSTEM:  DeviceAdapters
// AUTHOR:     Nico Stuurman
// COPYRIGHT:  Regents of the University of California, 2015
// 
//-----------------------------------------------------------------------------
// DESCRIPTION:
// Adapter for the Spectral/Andor/Oxford Instruments Diskovery 1 spinning disk confocal
// microscope system
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
  
#include "Diskovery.h"

// Diskovery commands
const char* g_VersionHWMajor = "Q:VERSION_HW_MAJOR";
const char* g_VersionHWMinor = "Q:VERSION_HW_MINOR";
const char* g_VersionHWRevision = "Q:VERSION_HW_REVISION";
const char* g_VersionFWMajor = "Q:VERSION_FW_MAJOR";
const char* g_VersionFWMinor = "Q:VERSION_FW_MINOR";
const char* g_VersionFWRevision = "Q:VERSION_FW_REVISION";
const char* g_ManufactureYear = "Q:MANUFACTURE_YEAR";
const char* g_ManufactureMonth = "Q:MANUFACTURE_MONTH";
const char* g_ManufactureDay = "Q:MANUFACTURE_DAY";
const char* g_ProductModel = "Q:PRODUCT_MODEL";
const char* g_SerialNumber = "Q:PRODUCT_SERIAL_NO";
const char* g_SetPresetSD = "A:PRESET_SD,";
const char* g_GetPresetSD = "Q:PRESET_SD";
const char* g_SetPresetWF = "A:PRESET_WF,";
const char* g_GetPresetWF = "Q:PRESET_WF";
const char* g_SetPresetFilterW = "A:PRESET_FILTER_W,";
const char* g_GetPresetFilterW = "Q:PRESET_FILTER_W";
const char* g_SetPresetFilterT = "A:PRESET_FILTER_T,";
const char* g_GetPresetFilterT = "Q:PRESET_FILTER_T";
const char* g_SetPresetIris = "A:PRESET_IRIS,";
const char* g_GetPresetIris = "Q:PRESET_IRIS";
const char* g_SetPresetTIRF = "A:PRESET_PX,";
const char* g_GetPresetTIRF = "Q:PRESET_PX";
const char* g_SetPositionRot = "A:POSITION_ROT,";
const char* g_GetPositionRot = "Q:POSITION_ROT";
const char* g_SetPositionLin = "A:POSITION_LIN,";
const char* g_GetPositionLin = "Q:POSITION_LIN";
const char* g_SetMotorRunningSD = "A:MOTOR_RUNNING_SD,";
const char* g_GetMotorRunningSD = "Q:MOTOR_RUNNING_SD";

// commands to determine configuration of this device
const char* g_HasWFX = "Q:ACTIVE_WF_X";
const char* g_HasWFY = "Q:ACTIVE_WF_Y";
const char* g_HasSD = "Q:ACTIVE_SD";
const char* g_HasROT = "Q:ACTIVE_ROT";
const char* g_HasLIN = "Q:ACTIVE_LIN";
const char* g_HasP1 = "Q:ACTIVE_P1";
const char* g_HasP2 = "Q:ACTIVE_P2";
const char* g_HasIRIS = "Q:ACTIVE_IRIS";
const char* g_HasFilterW = "Q:ACTIVE_FILTER_W";
const char* g_HasFilterT = "Q:ACTIVE_FILTER_T";

// commands to retrieve labels (as shown on the buttons
const char* g_BNamePart2 = "_NAME";
const char* g_ButtonWF = "Q:BUTTON_WF_";
const char* g_ButtonIris = "Q:BUTTON_IRIS_";
const char* g_ButtonFilterW = "Q:BUTTON_FILTER_W_";
const char* g_ButtonFilterT = "Q:BUTTON_FILTER_T_";
const char* g_ButtonDisk = "Q:DISK_PATTERN_NAME,";

// queries to get calibration data from the device
const char* g_TIRFFocalLength = "Q:TIRF_FOCAL_LENGTH"; // int32
const char* g_ResolutionLinear = "Q:RESOLUTION_LINEAR"; // double
const char* g_ResolutionRotation = "Q:RESOLUTION_ROTATION"; // double
const char* g_OffsetLinear = "Q:OFFSET_LINEAR"; //int32
const char* g_OffsetRotation = "Q:OFFSET_ROTATION"; //int32
const char* g_LineStart = "Q:LINE_"; // 0 < x < 8
const char* g_Wavelength = "_WAVELENGTH"; // int16
const char* g_Enabled = "_ENABLED"; // Boolean (0 or 1)

// commands to retrieve maximum and minimum positions of some devices
const char* g_MinCountLin = "Q:MIN_COUNT_LIN";  //uint32_t
const char* g_MaxCountLin = "Q:MAX_COUNT_LIN";  //uint32_t
const char* g_MinCountRot = "Q:MIN_COUNT_ROT";  //uint32_t
const char* g_MaxCountRot = "Q:MAX_COUNT_ROT";  //uint32_t

/**
 * Class that sends commands to the Diskovery1
 */
DiskoveryCommander::DiskoveryCommander(
      MM::Device& device, 
      MM::Core& core, 
      std::string serialPort, 
      DiskoveryModel* model) :
   device_(device),
   core_(core),
   model_(model),
   port_(serialPort)
{
   sender_ = new MessageSender(device_, core_, port_, blockingQueue_, model_);
}

DiskoveryCommander::~DiskoveryCommander()
{
   delete(sender_);
}   

/**
 * Queries the controller for its current settings.
 * Answer will be read by the Listening thread that should already 
 * have been started.
 * Sends the commands in batches of 3 with 50 ms waits in between
 * to avoid overlaoding the controller (this worked with the demo unit
 * I had, adjust if there are problems querying on startup).
 */
int DiskoveryCommander::Initialize()
{
   // Request the stuff that does not change
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_VersionHWMajor) );
   RETURN_ON_MM_ERROR( SendCommand(g_VersionHWMinor) );
   RETURN_ON_MM_ERROR( SendCommand(g_VersionHWRevision) );
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_VersionFWMajor) );
   RETURN_ON_MM_ERROR( SendCommand(g_VersionFWMinor) );
   RETURN_ON_MM_ERROR( SendCommand(g_VersionFWRevision) );
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_ManufactureYear) );
   RETURN_ON_MM_ERROR( SendCommand(g_ManufactureMonth) );
   RETURN_ON_MM_ERROR( SendCommand(g_ManufactureDay) );
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_SerialNumber) );
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetSD) );
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetWF) );
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetFilterW) );
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetFilterT) );
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetIris) );
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_GetPresetTIRF) );
   RETURN_ON_MM_ERROR( SendCommand(g_GetMotorRunningSD) );
   CDeviceUtils::SleepMs(50);

   // start looking at the blocking queue and send messages
   sender_->Start();

   return DEVICE_OK;
}

/**
 * Send request to the controller asking it to divulge what it 
 * can and can not do.  Ansers should be received and analyzed elsewhere
 */
int DiskoveryCommander::CheckCapabilities()
{
   CDeviceUtils::SleepMs(50);
   RETURN_ON_MM_ERROR( SendCommand(g_HasWFX) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasWFY) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasSD) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasROT) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasLIN) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasP1) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasP2) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasIRIS) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasFilterW) );
   CDeviceUtils::SleepMs(16);
   RETURN_ON_MM_ERROR( SendCommand(g_HasFilterT) );
   // give the device some time to actually send the command
   // so that other code can make use of the results
   CDeviceUtils::SleepMs(50);

   for (uint16_t i = 1; i < 5; i++) {
      if (model_->GetHasWFX()) {
         GetWFButtonName(i);
         CDeviceUtils::SleepMs(16);
      }
      if (model_->GetHasIRIS()) {
         GetIrisButtonName(i);
         CDeviceUtils::SleepMs(16);
      }
      if (model_->GetHasFilterW()) {
         GetFilterWButtonName(i);
         CDeviceUtils::SleepMs(16);
      }
      if (model_->GetHasFilterT()) {
         GetFilterTButtonName(i);
         CDeviceUtils::SleepMs(16);
      }
      if (model_->GetHasSD() && i < 4)
      {
         GetDiskButtonName(i);
         CDeviceUtils::SleepMs(16);
      }
   }

   // get initial position of some devices
   if (model_->GetHasLIN()) {
      RETURN_ON_MM_ERROR( SendCommand(g_MinCountLin) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_MaxCountLin) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_GetPositionLin) );
      CDeviceUtils::SleepMs(16);
   }
   if (model_->GetHasROT()) {
      RETURN_ON_MM_ERROR( SendCommand(g_MinCountRot) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_MaxCountRot) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_GetPositionRot) );
      CDeviceUtils::SleepMs(16);
   }

   if (model_->GetHasP1() && model_->GetHasP2() && model_->GetHasROT() && model_->GetHasLIN())
   {
      RETURN_ON_MM_ERROR( SendCommand(g_TIRFFocalLength) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_ResolutionRotation) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_OffsetLinear) );
      CDeviceUtils::SleepMs(16);
      RETURN_ON_MM_ERROR( SendCommand(g_OffsetRotation) );
      CDeviceUtils::SleepMs(16);
      for (uint16_t i = 1; i < 8; i++) 
      {
         std::ostringstream os;
         os << g_LineStart << i;
         RETURN_ON_MM_ERROR( SendCommand( (os.str() + g_Wavelength).c_str() ) );
         CDeviceUtils::SleepMs(16);
         RETURN_ON_MM_ERROR( SendCommand( (os.str() + g_Enabled).c_str() ) );
         CDeviceUtils::SleepMs(16);
      }
   }

   return DEVICE_OK;
}


/**
 * This function is used to send an inocous command
 */
int DiskoveryCommander::GetProductModel()
{
   RETURN_ON_MM_ERROR( SendCommand(g_ProductModel) );
   return DEVICE_OK;
}

int DiskoveryCommander::SetPresetSD(uint16_t pos)
{
   return SendSetCommand(g_SetPresetSD, pos);
}

int DiskoveryCommander::SetPresetWF(uint16_t pos)
{
   return SendSetCommand(g_SetPresetWF, pos);
}

int DiskoveryCommander::SetPresetFilterW(uint16_t pos)
{
   return SendSetCommand(g_SetPresetFilterW, pos);
}

int DiskoveryCommander::SetPresetFilterT(uint16_t pos)
{
   return SendSetCommand(g_SetPresetFilterT, pos);
}
int DiskoveryCommander::SetPresetIris(uint16_t pos)
{
   return SendSetCommand(g_SetPresetIris, pos);
}

int DiskoveryCommander::SetPresetTIRF(uint16_t pos)
{
   return SendSetCommand(g_SetPresetTIRF, pos);
}

int DiskoveryCommander::SetPositionLin(uint32_t pos)
{
   return SendSetCommand(g_SetPositionLin, pos);
}

int DiskoveryCommander::SetPositionRot(uint32_t pos)
{
   return SendSetCommand(g_SetPositionRot, pos);
}

int DiskoveryCommander::SetMotorRunningSD(uint16_t pos)
{
   return SendSetCommand(g_SetMotorRunningSD, pos);
}

int DiskoveryCommander::GetWFButtonName(uint16_t pos) {
   if (pos > 0 && pos < 5)
      return SendSetCommand(g_ButtonWF, pos, g_BNamePart2);
   return ERR_UNKNOWN_POSITION;
}

int DiskoveryCommander::GetIrisButtonName(uint16_t pos) {
   if (pos > 0 && pos < 5)
      return SendSetCommand(g_ButtonIris, pos, g_BNamePart2);
   return ERR_UNKNOWN_POSITION;
}

int DiskoveryCommander::GetFilterWButtonName(uint16_t pos) {
   if (pos > 0 && pos < 5)
      return SendSetCommand(g_ButtonFilterW, pos, g_BNamePart2);
   return ERR_UNKNOWN_POSITION;
}

int DiskoveryCommander::GetFilterTButtonName(uint16_t pos) {
   if (pos > 0 && pos < 5)
      return SendSetCommand(g_ButtonFilterT, pos, g_BNamePart2);
   return ERR_UNKNOWN_POSITION;
}

int DiskoveryCommander::GetDiskButtonName(uint16_t pos) {
   if (pos > 0 && pos < 4)
   {
      std::ostringstream os;
      os << g_ButtonDisk << pos;
      return SendCommand(os.str().c_str());
   }
   return ERR_UNKNOWN_POSITION;
}

int DiskoveryCommander::SendTIRFGOTO() 
{
   double depth = model_->GetDepth();
   if (depth < 50.0)
      depth = 50.0;
   if (depth > 1000.0)
      depth = 1000.0;
   std::ostringstream os;
   os << "A:TIRF_GOTO,";
   os << model_->GetNA() << "~" << model_->GetTubeLensFocalLength() << "~";
   os << model_->GetOM() << "~" << model_->GetRI() << "~";
   os << depth << "~" << model_->GetWavelength1() << "~";
   os << model_->GetWavelength2() << "~";
   if (model_->GetExitTIRF())
      os << "1";
   else
      os << "0";
    model_->SetLogicalBusy(true);
    blockingQueue_.push(os.str());

   return DEVICE_OK;
}

int DiskoveryCommander::SendSetCommand(const char* commandPart1, uint16_t pos, const char* commandPart2) 
{
   std::ostringstream os;
   os << commandPart1 << pos << commandPart2;
   SendCommand(os.str().c_str());

   return DEVICE_OK;
}

/**
 * Do not send commands directly, but queue them up
 * They will be send by the MessageSender whenever the device is not busy
 */
int DiskoveryCommander::SendSetCommand(const char* command, uint16_t pos)
{
   model_->SetLogicalBusy(true);
   std::ostringstream os;
   os << command << pos;
   blockingQueue_.push(os.str());

   return DEVICE_OK;
}

/**
 * Do not send commands directly, but queue them up
 * They will be send by the MessageSender whenever the device is not busy
 */
int DiskoveryCommander::SendSetCommand(const char* command, uint32_t pos)
{
   model_->SetLogicalBusy(true);
   std::ostringstream os;
   os << command << pos;
   blockingQueue_.push(os.str());

   return DEVICE_OK;
}

int DiskoveryCommander::SendCommand(const char* command)
{
   RETURN_ON_MM_ERROR(  core_.SetSerialCommand(&device_, port_.c_str(), command, "\n") );

   return DEVICE_OK;
}

