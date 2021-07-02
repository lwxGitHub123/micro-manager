/*
+-----------------------------------------------------------------+
| DSLRRemoteLib.h                                                 |
+-----------------------------------------------------------------+
|                                                                 |
| Description:                                                    |
|   API for DSLRRemoteLib interface library for interfacing to    |
|   DSLRRemote and DSLR Remote Pro.                               |
|                                                                 |
| Copyright (c) 2004-2007, Breeze Systems Limited                 |
| www.breezesys.com                                               |
+-----------------------------------------------------------------+
*/
#ifdef DSLRREMOTELIB_EXPORTS
#define DSLRREMOTELIB_API __declspec(dllexport)
#else
#define DSLRREMOTELIB_API __declspec(dllimport)
#endif

extern "C"
{

//-----------------------------------------------------------------------
// PingCamera()
// Inputs: none
//
// Returns:
//   0 - Success, camera is connected and ready
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//
// Description:
//   Test to see if DSLR Remote is running and whether the camera
//   is ready to take a photo.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall PingCamera();

//-----------------------------------------------------------------------
// GetCameraModel()
// Inputs:
//   pszModel        string in which to store the camera model name
//   numChars        length of pszModel
//
// Returns:
//   0 - Success, camera model returned in pszModel
//   1 - DSLR Remote is not running
//   4 - Some other error
//
// Description:
//   Returns the name of the camera model DSLR Remote Pro is connected to.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall GetCameraModel(
							char* pszModel,
							int   numChars
							);

//-----------------------------------------------------------------------
// ReleaseShutter()
// Inputs:
//   timeOutInSecs   timeout in secs to wait for picture to be
//                   taken and downloaded (max 60 secs)
//   pszFilename     optional string in which to store the name of the
//                   saved image. Set to NULL if not required
//   numChars        length of pszFilename if defined
//
// Returns:
//   0 - Success, image saved
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Timeout waiting for image to be saved
//   5 - Error releasing shutter e.g. AF failure
//
// Description:
//   Take a picture and optionally wait for it to be saved to disk.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall ReleaseShutter(
							int   timeoutInSecs,
							char* pszFilename,
							int   numChars
							);


//-----------------------------------------------------------------------
// SetZoomPosition()
// This is only for compatability with PSRemoteLib as it isn't
// possible to zoom Canon EOS DSLR lenses remotely
//
// Inputs:
//   zoomPos	Position to set zoom lens
//
// Returns:
//   0 - DSLR Remote is not running
//   1 - DSLR Remote is running but camera is not connected
//   2 - Camera is busy
//   6 - Success
//   7 - Some other error
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetZoomPosition(int zoomPos);

//-----------------------------------------------------------------------
// SetISO()
// Inputs:
//   iso		ISO setting, numbered from 0 in the same
//              order as the ISO dropdown list.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetISO(int iso);

//-----------------------------------------------------------------------
// SetImageSizeQuality()
// Inputs:
//   image		Image size and quality setting, numbered from 0 in the same
//              order as the image size/quality dropdown list.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetImageSizeQuality(int image);

//-----------------------------------------------------------------------
// SetExposureMode()
// Inputs:
//   mode		Exposure mode setting, numbered from 0 in the same
//              order as the exposure mode dropdown list. Currently only
//				the Canon EOS-1D and Canon EOS-1DS allow the exposure
//				mode to be set remotely. Other EOS cameras require the
//				exposure mode to be set by turning the physical dial
//				on the camera.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetExposureMode(int mode);

//-----------------------------------------------------------------------
// SetExposureCompensation()
// Inputs:
//   comp		Exposure compensation, numbered from 0 in the same
//              order as the exposure compensation dropdown list.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetExposureCompensation(int comp);

//-----------------------------------------------------------------------
// SetShutterAperture()
// Inputs:
//   shutter	Shutter speed, numbered from 0 in the same
//              order as the shutter speed dropdown list. A value
//              of -1 leaves the shutter speed unchanged
//   aperture	Aperture, numbered from 0 in the same
//              order as the aperture dropdown list. A value
//              of -1 leaves the aperture unchanged
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetShutterAperture(int shutter, int aperture);

//-----------------------------------------------------------------------
// SetWhiteBalance()
// Inputs:
//   wb     	White balance, numbered from 0 in the same
//              order as the white balance dropdown list. Set this to
//				the color temperature to select kelvin white balance
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetWhiteBalance(int wb);

//-----------------------------------------------------------------------
// DisplayEVF()
// Inputs:
//   display     display or hide live view (EVF) window
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error e.g. EVF not supported
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall DisplayEVF(bool display);

//-----------------------------------------------------------------------
// SetLensFocus()
// Inputs:
//   focus     	Size of focus step:
//              -3 = near, large step
//              -2 = near, medium step
//              -1 = near, small step
//               1 = far, small step
//               2 = far, medium step
//               3 = far, large step
//	Note: EVF must be displayed before calling this function
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error e.g. EVF not supported or not active
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetLensFocus(int focus);

//-----------------------------------------------------------------------
// SetFilenamePrefix()
// Inputs:
//   pszPrefix       zero-terminated string specifying the filename
//                   prefix.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   3 - Camera is busy
//
// Description:
//   Defines the prefix used for filenames when saving images to disk
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetFilenamePrefix(
							const char* pszPrefix
							);

//-----------------------------------------------------------------------
// SetOutputPath()
// Inputs:
//   pszPathname     zero-terminated string specifying the directory in
//                   which to save images.
//
// Returns:
//   0 - Success, pathname returned in pszPathname
//   1 - DSLR Remote is not running
//   3 - Camera is busy
//   8 - Error creating directory
//
// Description:
//   Defines the directory in which to store images. If the directory does
//   not exist this function will try to create it and return an error if
//   it fails.
//   NOTE: This function only sets the base part of the directory. The actual
//   directory in which images are saved will be be different if DSLR Remote
//   preferences are set to use a separate directory for the year, month or day.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetOutputPath(
							const char* pszPathname
							);


//-----------------------------------------------------------------------
// GetOutputPath()
// Inputs:
//   pszPathname     string in which to store the pathname of the
//                   directory currently being used to save images
//   numChars        length of pszPathname
//
// Returns:
//   0 - Success, pathname returned in pszPathname
//   1 - DSLR Remote is not running
//   4 - Some other error
//
// Description:
//   Returns the full pathname of the directory used for saving images.
//   This is the base directory as specified by SetOutputPath() plus
//   any separate directories for year, month or day if selected in
//   preferences.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall GetOutputPath(
							char* pszPathname,
							int   numChars
							);


//-----------------------------------------------------------------------
// SetComment()
// Inputs:
//   pszComment      zero-terminated string specifying the comment to be
//                   stored in the EXIF data of pictures.
//                   Max length 255 characters.
//
// Returns:
//   0 - Success, pathname returned in pszPathname
//   1 - DSLR Remote is not running
//   3 - Camera is busy
//   4 - Some other error
//   9 - String too long (max length 255 characters)
//
// Description:
//   Defines the comment string to be stored in the EXIF data of pictures
//   when they are taken. This function updates the "Comment" edit box in
//   the DSLR Remote window and does not affect the currently displayed image.
//
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SetComment(
							const char* pszComment
							);

};

//-----------------------------------------------------------------------
// ConnectDisconnect()
// Inputs:
//   connect	    Attempt to connect to the camera if true,
//				    disconnect if false
//   timeoutInSecs  Connection timeout in secs. Max 600 secs, recommended
//                  minimum setting of 15 secs.
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   2 - DSLR Remote is running but camera is not connected
//   3 - Camera is busy
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall ConnectDisconnect(bool connect, int timeoutInSecs);

//-----------------------------------------------------------------------
// SelectCamera() - only applicable for multiple camera versions of DSLR Remote Pro
// Inputs:
//   camera		Camera number starting from 0
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall SelectCamera(int camera);

//-----------------------------------------------------------------------
// ExitApp()
// Inputs:
//   connect	Disconnect from the camera and exit DSLR Remote Pro
//
// Returns:
//   0 - Success
//   1 - DSLR Remote is not running
//   4 - Some other error
//-----------------------------------------------------------------------
DSLRREMOTELIB_API int __stdcall ExitApp();
