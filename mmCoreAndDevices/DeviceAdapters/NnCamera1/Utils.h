#pragma once
//#pragma once
///////////////////////////////////////////////////////////////////////////////
// FILE:          Common.h
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

#ifndef _UTILS_H_
#define _UTILS_H_



class Utils
{
public:

   Utils(void);
   ~Utils(void);

   string getCwd();
   string createDirectory(string directoryPath );
   void cameraLog(string log);

private:
  static string mpath;
};




#endif //_UTILS_H_

