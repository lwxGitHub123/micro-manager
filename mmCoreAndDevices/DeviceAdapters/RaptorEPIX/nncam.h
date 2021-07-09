#ifndef __nncam_h__
#define __nncam_h__

/* Version: 49.18939.20210514 */
/*
   Platform & Architecture:
       (1) Win32:
              (a) x86: XP SP3 or above; CPU supports SSE2 instruction set or above
              (b) x64: Win7 or above
              (c) arm: Win10 or above
              (d) arm64: Win10 or above
       (2) WinRT: x86, x64, arm, arm64; Win10 or above
       (3) macOS: universal (x64 + x86); macOS 10.10 or above
       (4) Linux: kernel 2.6.27 or above; GLIBC 2.17 or above
              (a) x86: CPU supports SSE3 instruction set or above; built by gcc 5.4.0
              (b) x64: built by gcc 5.4.0
              (c) armel: built by toolchain arm-linux-gnueabi (version 4.9.2)
              (d) armhf: built by toolchain arm-linux-gnueabihf (version 4.9.2)
              (e) arm64: built by toolchain aarch64-linux-gnu (version 4.9.2)
       (5) Android: arm, arm64, x86, x64; built by android-ndk-r18b; __ANDROID_API__ = 23
*/
/*
    doc:
       (1) en.html, English
       (2) hans.html, Simplified Chinese
*/

#ifdef _WIN32
#ifndef _INC_WINDOWS
#include <windows.h>
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus) && (__cplusplus >= 201402L)
#define NNCAM_DEPRECATED  [[deprecated]]
#elif defined(_MSC_VER)
#define NNCAM_DEPRECATED  __declspec(deprecated)
#elif defined(__GNUC__) || defined(__clang__)
#define NNCAM_DEPRECATED  __attribute__((deprecated))
#else
#define NNCAM_DEPRECATED
#endif

#ifdef _WIN32 /* Windows */

#pragma pack(push, 8)
#ifdef NNCAM_EXPORTS
#define NNCAM_API(x)    __declspec(dllexport)   x   __stdcall  /* in Windows, we use __stdcall calling convention, see https://docs.microsoft.com/en-us/cpp/cpp/stdcall */
#elif !defined(NNCAM_NOIMPORTS)
#define NNCAM_API(x)    __declspec(dllimport)   x   __stdcall
#else
#define NNCAM_API(x)    x   __stdcall
#endif

#else   /* Linux or macOS */

#define NNCAM_API(x)    x
#if (!defined(HRESULT)) && (!defined(__COREFOUNDATION_CFPLUGINCOM__)) /* CFPlugInCOM.h */
#define HRESULT int
#endif
#ifndef SUCCEEDED
#define SUCCEEDED(hr)   (((HRESULT)(hr)) >= 0)
#endif
#ifndef FAILED
#define FAILED(hr)      (((HRESULT)(hr)) < 0)
#endif

#ifndef __stdcall
#define __stdcall
#endif

#ifndef __BITMAPINFOHEADER_DEFINED__
#define __BITMAPINFOHEADER_DEFINED__
typedef struct {
    unsigned        biSize;
    int             biWidth;
    int             biHeight;
    unsigned short  biPlanes;
    unsigned short  biBitCount;
    unsigned        biCompression;
    unsigned        biSizeImage;
    int             biXPelsPerMeter;
    int             biYPelsPerMeter;
    unsigned        biClrUsed;
    unsigned        biClrImportant;
} BITMAPINFOHEADER;
#endif

#ifndef __RECT_DEFINED__
#define __RECT_DEFINED__
typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} RECT, *PRECT;
#endif

#endif

#ifndef TDIBWIDTHBYTES
#define TDIBWIDTHBYTES(bits)  ((unsigned)(((bits) + 31) & (~31)) / 8)
#endif

/*****************************************************************************************************************/
/* HRESULT                                                                                                       */
/*    |----------------|------------------------------------------------------------------------|------------|   */
/*    | S_OK           |   Operation successful                                                 | 0x00000000 |   */
/*    | S_FALSE        |   Operation successful                                                 | 0x00000001 |   */
/*    | E_FAIL         |   Unspecified failure                                                  | 0x80004005 |   */
/*    | E_ACCESSDENIED |   General access denied error                                          | 0x80070005 |   */
/*    | E_INVALIDARG   |   One or more arguments are not valid                                  | 0x80070057 |   */
/*    | E_NOTIMPL      |   Not supported or not implemented                                     | 0x80004001 |   */
/*    | E_NOINTERFACE  |   Interface not supported                                              | 0x80004002 |   */
/*    | E_POINTER      |   Pointer that is not valid                                            | 0x80004003 |   */
/*    | E_UNEXPECTED   |   Unexpected failure                                                   | 0x8000FFFF |   */
/*    | E_OUTOFMEMORY  |   Out of memory                                                        | 0x8007000E |   */
/*    | E_WRONG_THREAD |   call function in the wrong thread                                    | 0x8001010E |   */
/*    | E_GEN_FAILURE  |   device not functioning                                               | 0x8007001F |   */
/*    | E_PENDING      |   The data necessary to complete this operation is not yet available   | 0x8000000A |   */
/*    |----------------|------------------------------------------------------------------------|------------|   */
/*****************************************************************************************************************/
/*                                                                              */
/* Please note that the return value >= 0 means success                         */
/* (especially S_FALSE is also successful, indicating that the internal value and the value set by the user is equivalent, which means "no operation"). */
/* Therefore, the SUCCEEDEDand FAILED macros should generally be used to determine whether the return value is successful or failed. */
/* (Unless there are special needs, do not use "==S_OK" or "==0" to judge the return value) */
/*                                                                              */
/* #define SUCCEEDED(hr)   (((HRESULT)(hr)) >= 0)                               */
/* #define FAILED(hr)      (((HRESULT)(hr)) < 0)                                */
/*                                                                              */
/********************************************************************************/

/* handle */
typedef struct NncamT { int unused; } *HNncam;

#define NNCAM_MAX                      16
                                         
#define NNCAM_FLAG_CMOS                0x00000001  /* cmos sensor */
#define NNCAM_FLAG_CCD_PROGRESSIVE     0x00000002  /* progressive ccd sensor */
#define NNCAM_FLAG_CCD_INTERLACED      0x00000004  /* interlaced ccd sensor */
#define NNCAM_FLAG_ROI_HARDWARE        0x00000008  /* support hardware ROI */
#define NNCAM_FLAG_MONO                0x00000010  /* monochromatic */
#define NNCAM_FLAG_BINSKIP_SUPPORTED   0x00000020  /* support bin/skip mode, see Nncam_put_Mode and Nncam_get_Mode */
#define NNCAM_FLAG_USB30               0x00000040  /* usb3.0 */
#define NNCAM_FLAG_TEC                 0x00000080  /* Thermoelectric Cooler */
#define NNCAM_FLAG_USB30_OVER_USB20    0x00000100  /* usb3.0 camera connected to usb2.0 port */
#define NNCAM_FLAG_ST4                 0x00000200  /* ST4 port */
#define NNCAM_FLAG_GETTEMPERATURE      0x00000400  /* support to get the temperature of the sensor */
#define NNCAM_FLAG_RAW10               0x00001000  /* pixel format, RAW 10bits */
#define NNCAM_FLAG_RAW12               0x00002000  /* pixel format, RAW 12bits */
#define NNCAM_FLAG_RAW14               0x00004000  /* pixel format, RAW 14bits */
#define NNCAM_FLAG_RAW16               0x00008000  /* pixel format, RAW 16bits */
#define NNCAM_FLAG_FAN                 0x00010000  /* cooling fan */
#define NNCAM_FLAG_TEC_ONOFF           0x00020000  /* Thermoelectric Cooler can be turn on or off, support to set the target temperature of TEC */
#define NNCAM_FLAG_ISP                 0x00040000  /* ISP (Image Signal Processing) chip */
#define NNCAM_FLAG_TRIGGER_SOFTWARE    0x00080000  /* support software trigger */
#define NNCAM_FLAG_TRIGGER_EXTERNAL    0x00100000  /* support external trigger */
#define NNCAM_FLAG_TRIGGER_SINGLE      0x00200000  /* only support trigger single: one trigger, one image */
#define NNCAM_FLAG_BLACKLEVEL          0x00400000  /* support set and get the black level */
#define NNCAM_FLAG_AUTO_FOCUS          0x00800000  /* support auto focus */
#define NNCAM_FLAG_BUFFER              0x01000000  /* frame buffer */
#define NNCAM_FLAG_DDR                 0x02000000  /* use very large capacity DDR (Double Data Rate SDRAM) for frame buffer */
#define NNCAM_FLAG_CG                  0x04000000  /* Conversion Gain: HCG, LCG */
#define NNCAM_FLAG_YUV411              0x08000000  /* pixel format, yuv411 */
#define NNCAM_FLAG_VUYY                0x10000000  /* pixel format, yuv422, VUYY */
#define NNCAM_FLAG_YUV444              0x20000000  /* pixel format, yuv444 */
#define NNCAM_FLAG_RGB888              0x40000000  /* pixel format, RGB888 */
#define NNCAM_FLAG_RAW8                0x80000000  /* pixel format, RAW 8 bits */
#define NNCAM_FLAG_GMCY8               0x0000000100000000  /* pixel format, GMCY, 8bits */
#define NNCAM_FLAG_GMCY12              0x0000000200000000  /* pixel format, GMCY, 12bits */
#define NNCAM_FLAG_UYVY                0x0000000400000000  /* pixel format, yuv422, UYVY */
#define NNCAM_FLAG_CGHDR               0x0000000800000000  /* Conversion Gain: HCG, LCG, HDR */
#define NNCAM_FLAG_GLOBALSHUTTER       0x0000001000000000  /* global shutter */
#define NNCAM_FLAG_FOCUSMOTOR          0x0000002000000000  /* support focus motor */
#define NNCAM_FLAG_PRECISE_FRAMERATE   0x0000004000000000  /* support precise framerate & bandwidth, see NNCAM_OPTION_PRECISE_FRAMERATE & NNCAM_OPTION_BANDWIDTH */
#define NNCAM_FLAG_HEAT                0x0000008000000000  /* support heat to prevent fogging up */
#define NNCAM_FLAG_LOW_NOISE           0x0000010000000000  /* support low noise mode (Higher signal noise ratio, lower frame rate) */
#define NNCAM_FLAG_LEVELRANGE_HARDWARE 0x0000020000000000  /* hardware level range, put(get)_LevelRangeV2 */
#define NNCAM_FLAG_EVENT_HARDWARE      0x0000040000000000  /* hardware event, such as exposure start & stop */

#define NNCAM_TEMP_DEF                 6503    /* temp, default */
#define NNCAM_TEMP_MIN                 2000    /* temp, minimum */
#define NNCAM_TEMP_MAX                 15000   /* temp, maximum */
#define NNCAM_TINT_DEF                 1000    /* tint */
#define NNCAM_TINT_MIN                 200     /* tint */
#define NNCAM_TINT_MAX                 2500    /* tint */
#define NNCAM_HUE_DEF                  0       /* hue */
#define NNCAM_HUE_MIN                  (-180)  /* hue */
#define NNCAM_HUE_MAX                  180     /* hue */
#define NNCAM_SATURATION_DEF           128     /* saturation */
#define NNCAM_SATURATION_MIN           0       /* saturation */
#define NNCAM_SATURATION_MAX           255     /* saturation */
#define NNCAM_BRIGHTNESS_DEF           0       /* brightness */
#define NNCAM_BRIGHTNESS_MIN           (-64)   /* brightness */
#define NNCAM_BRIGHTNESS_MAX           64      /* brightness */
#define NNCAM_CONTRAST_DEF             0       /* contrast */
#define NNCAM_CONTRAST_MIN             (-100)  /* contrast */
#define NNCAM_CONTRAST_MAX             100     /* contrast */
#define NNCAM_GAMMA_DEF                100     /* gamma */
#define NNCAM_GAMMA_MIN                20      /* gamma */
#define NNCAM_GAMMA_MAX                180     /* gamma */
#define NNCAM_AETARGET_DEF             120     /* target of auto exposure */
#define NNCAM_AETARGET_MIN             16      /* target of auto exposure */
#define NNCAM_AETARGET_MAX             220     /* target of auto exposure */
#define NNCAM_WBGAIN_DEF               0       /* white balance gain */
#define NNCAM_WBGAIN_MIN               (-127)  /* white balance gain */
#define NNCAM_WBGAIN_MAX               127     /* white balance gain */
#define NNCAM_BLACKLEVEL_MIN           0       /* minimum black level */
#define NNCAM_BLACKLEVEL8_MAX          31              /* maximum black level for bit depth = 8 */
#define NNCAM_BLACKLEVEL10_MAX         (31 * 4)        /* maximum black level for bit depth = 10 */
#define NNCAM_BLACKLEVEL12_MAX         (31 * 16)       /* maximum black level for bit depth = 12 */
#define NNCAM_BLACKLEVEL14_MAX         (31 * 64)       /* maximum black level for bit depth = 14 */
#define NNCAM_BLACKLEVEL16_MAX         (31 * 256)      /* maximum black level for bit depth = 16 */
#define NNCAM_SHARPENING_STRENGTH_DEF  0       /* sharpening strength */
#define NNCAM_SHARPENING_STRENGTH_MIN  0       /* sharpening strength */
#define NNCAM_SHARPENING_STRENGTH_MAX  500     /* sharpening strength */
#define NNCAM_SHARPENING_RADIUS_DEF    2       /* sharpening radius */
#define NNCAM_SHARPENING_RADIUS_MIN    1       /* sharpening radius */
#define NNCAM_SHARPENING_RADIUS_MAX    10      /* sharpening radius */
#define NNCAM_SHARPENING_THRESHOLD_DEF 0       /* sharpening threshold */
#define NNCAM_SHARPENING_THRESHOLD_MIN 0       /* sharpening threshold */
#define NNCAM_SHARPENING_THRESHOLD_MAX 255     /* sharpening threshold */
#define NNCAM_AUTOEXPO_THRESHOLD_DEF   5       /* auto exposure threshold */
#define NNCAM_AUTOEXPO_THRESHOLD_MIN   2       /* auto exposure threshold */
#define NNCAM_AUTOEXPO_THRESHOLD_MAX   15      /* auto exposure threshold */
#define NNCAM_BANDWIDTH_DEF            90      /* bandwidth */
#define NNCAM_BANDWIDTH_MIN            1       /* bandwidth */
#define NNCAM_BANDWIDTH_MAX            100     /* bandwidth */
#define NNCAM_DENOISE_DEF              0       /* denoise */
#define NNCAM_DENOISE_MIN              0       /* denoise */
#define NNCAM_DENOISE_MAX              100     /* denoise */

typedef struct{
    unsigned    width;
    unsigned    height;
}NncamResolution;

/* In Windows platform, we always use UNICODE wchar_t */
/* In Linux or macOS, we use char */

typedef struct {
#ifdef _WIN32
    const wchar_t*      name;        /* model name, in Windows, we use unicode */
#else
    const char*         name;        /* model name */
#endif
    unsigned long long  flag;        /* NNCAM_FLAG_xxx, 64 bits */
    unsigned            maxspeed;    /* number of speed level, same as Nncam_get_MaxSpeed(), the speed range = [0, maxspeed], closed interval */
    unsigned            preview;     /* number of preview resolution, same as Nncam_get_ResolutionNumber() */
    unsigned            still;       /* number of still resolution, same as Nncam_get_StillResolutionNumber() */
    unsigned            maxfanspeed; /* maximum fan speed */
    unsigned            ioctrol;     /* number of input/output control */
    float               xpixsz;      /* physical pixel size */
    float               ypixsz;      /* physical pixel size */
    NncamResolution   res[NNCAM_MAX];
}NncamModelV2; /* camera model v2 */

typedef struct {
#ifdef _WIN32
    wchar_t               displayname[64];    /* display name */
    wchar_t               id[64];             /* unique and opaque id of a connected camera, for Nncam_Open */
#else
    char                  displayname[64];    /* display name */
    char                  id[64];             /* unique and opaque id of a connected camera, for Nncam_Open */
#endif
    const NncamModelV2* model;
}NncamDeviceV2; /* camera instance for enumerating */

/*
    get the version of this dll/so/dylib, which is: 49.18939.20210514
*/
#ifdef _WIN32
NNCAM_API(const wchar_t*)   Nncam_Version();
#else
NNCAM_API(const char*)      Nncam_Version();
#endif

/*
    enumerate the cameras connected to the computer, return the number of enumerated.

    NncamDeviceV2 arr[NNCAM_MAX];
    unsigned cnt = Nncam_EnumV2(arr);
    for (unsigned i = 0; i < cnt; ++i)
        ...

    if pti == NULL, then, only the number is returned.
    Nncam_Enum is obsolete.
*/
NNCAM_API(unsigned) Nncam_EnumV2(NncamDeviceV2 pti[NNCAM_MAX]);

/* use the id of NncamDeviceV2, which is enumerated by Nncam_EnumV2.
    if id is NULL, Nncam_Open will open the first camera.
*/
#ifdef _WIN32
NNCAM_API(HNncam) Nncam_Open(const wchar_t* id);
#else
NNCAM_API(HNncam) Nncam_Open(const char* id);
#endif

/*
    the same with Nncam_Open, but use the index as the parameter. such as:
    index == 0, open the first camera,
    index == 1, open the second camera,
    etc
*/
NNCAM_API(HNncam) Nncam_OpenByIndex(unsigned index);

NNCAM_API(void)     Nncam_Close(HNncam h);  /* close the handle */

#define NNCAM_EVENT_EXPOSURE          0x0001    /* exposure time or gain changed */
#define NNCAM_EVENT_TEMPTINT          0x0002    /* white balance changed, Temp/Tint mode */
#define NNCAM_EVENT_IMAGE             0x0004    /* live image arrived, use Nncam_PullImage to get this image */
#define NNCAM_EVENT_STILLIMAGE        0x0005    /* snap (still) frame arrived, use Nncam_PullStillImage to get this frame */
#define NNCAM_EVENT_WBGAIN            0x0006    /* white balance changed, RGB Gain mode */
#define NNCAM_EVENT_TRIGGERFAIL       0x0007    /* trigger failed */
#define NNCAM_EVENT_BLACK             0x0008    /* black balance changed */
#define NNCAM_EVENT_FFC               0x0009    /* flat field correction status changed */
#define NNCAM_EVENT_DFC               0x000a    /* dark field correction status changed */
#define NNCAM_EVENT_ROI               0x000b    /* roi changed */
#define NNCAM_EVENT_LEVELRANGE        0x000c    /* level range changed */
#define NNCAM_EVENT_ERROR             0x0080    /* generic error */
#define NNCAM_EVENT_DISCONNECTED      0x0081    /* camera disconnected */
#define NNCAM_EVENT_NOFRAMETIMEOUT    0x0082    /* no frame timeout error */
#define NNCAM_EVENT_AFFEEDBACK        0x0083    /* auto focus feedback information */
#define NNCAM_EVENT_AFPOSITION        0x0084    /* auto focus sensor board positon */
#define NNCAM_EVENT_NOPACKETTIMEOUT   0x0085    /* no packet timeout */
#define NNCAM_EVENT_EXPO_START        0x4000    /* exposure start */
#define NNCAM_EVENT_EXPO_STOP         0x4001    /* exposure stop */
#define NNCAM_EVENT_TRIGGER_ALLOW     0x4002    /* next trigger allow */
#define NNCAM_EVENT_FACTORY           0x8001    /* restore factory settings */

#ifdef _WIN32
NNCAM_API(HRESULT)  Nncam_StartPullModeWithWndMsg(HNncam h, HWND hWnd, UINT nMsg);
#endif

/* Do NOT call Nncam_Close, Nncam_Stop in this callback context, it deadlocks. */
typedef void (__stdcall* PNNCAM_EVENT_CALLBACK)(unsigned nEvent, void* pCallbackCtx);
NNCAM_API(HRESULT)  Nncam_StartPullModeWithCallback(HNncam h, PNNCAM_EVENT_CALLBACK pEventCallback, void* pCallbackContext);

#define NNCAM_FRAMEINFO_FLAG_SEQ          0x01 /* sequence number */
#define NNCAM_FRAMEINFO_FLAG_TIMESTAMP    0x02 /* timestamp */

typedef struct {
    unsigned            width;
    unsigned            height;
    unsigned            flag;       /* NNCAM_FRAMEINFO_FLAG_xxxx */
    unsigned            seq;        /* sequence number */
    unsigned long long  timestamp;  /* microsecond */
}NncamFrameInfoV2;

/*
    bits: 24 (RGB24), 32 (RGB32), 48 (RGB48), 8 (Gray) or 16 (Gray). In RAW mode, this parameter is ignored.
    pnWidth, pnHeight: OUT parameter
    rowPitch: The distance from one row to the next row. rowPitch = 0 means using the default row pitch. rowPitch = -1 means zero padding
    
    -------------------------------------------------------------------------------------------------------------
    | format                                            | 0 means default row pitch     | -1 means zero padding |
    |---------------------------------------------------|-------------------------------|-----------------------|
    | RGB       | RGB24                                 | TDIBWIDTHBYTES(24 * Width)    | Width * 3             |
    |           | RGB32                                 | Width * 4                     | Width * 4             |
    |           | RGB48                                 | TDIBWIDTHBYTES(48 * Width)    | Width * 6             |
    |           | GREY8                                 | TDIBWIDTHBYTES(8 * Width)     | Width                 |
    |           | GREY16                                | TDIBWIDTHBYTES(16 * Width)    | Width * 2             |
    |-----------|---------------------------------------|-------------------------------|-----------------------|
    | RAW       | 8bits Mode                            | Width                         | Width                 |
    |           | 10bits, 12bits, 14bits, 16bits Mode   | Width * 2                     | Width * 2             |
    |-----------|---------------------------------------|-------------------------------|-----------------------|
*/
NNCAM_API(HRESULT)  Nncam_PullImageV2(HNncam h, void* pImageData, int bits, NncamFrameInfoV2* pInfo);
NNCAM_API(HRESULT)  Nncam_PullStillImageV2(HNncam h, void* pImageData, int bits, NncamFrameInfoV2* pInfo);
NNCAM_API(HRESULT)  Nncam_PullImageWithRowPitchV2(HNncam h, void* pImageData, int bits, int rowPitch, NncamFrameInfoV2* pInfo);
NNCAM_API(HRESULT)  Nncam_PullStillImageWithRowPitchV2(HNncam h, void* pImageData, int bits, int rowPitch, NncamFrameInfoV2* pInfo);

NNCAM_API(HRESULT)  Nncam_PullImage(HNncam h, void* pImageData, int bits, unsigned* pnWidth, unsigned* pnHeight);
NNCAM_API(HRESULT)  Nncam_PullStillImage(HNncam h, void* pImageData, int bits, unsigned* pnWidth, unsigned* pnHeight);
NNCAM_API(HRESULT)  Nncam_PullImageWithRowPitch(HNncam h, void* pImageData, int bits, int rowPitch, unsigned* pnWidth, unsigned* pnHeight);
NNCAM_API(HRESULT)  Nncam_PullStillImageWithRowPitch(HNncam h, void* pImageData, int bits, int rowPitch, unsigned* pnWidth, unsigned* pnHeight);

/*
    (NULL == pData) means that something is error
    pCallbackCtx is the callback context which is passed by Nncam_StartPushModeV3
    bSnap: TRUE if Nncam_Snap

    pDataCallback is callbacked by an internal thread of nncam.dll, so please pay attention to multithread problem.
    Do NOT call Nncam_Close, Nncam_Stop in this callback context, it deadlocks.
*/
typedef void (__stdcall* PNNCAM_DATA_CALLBACK_V3)(const void* pData, const NncamFrameInfoV2* pInfo, int bSnap, void* pCallbackCtx);
NNCAM_API(HRESULT)  Nncam_StartPushModeV3(HNncam h, PNNCAM_DATA_CALLBACK_V3 pDataCallback, void* pDataCallbackCtx, PNNCAM_EVENT_CALLBACK pEventCallback, void* pEventCallbackContext);

NNCAM_API(HRESULT)  Nncam_Stop(HNncam h);
NNCAM_API(HRESULT)  Nncam_Pause(HNncam h, int bPause);

/*  for pull mode: NNCAM_EVENT_STILLIMAGE, and then Nncam_PullStillImage
    for push mode: the snapped image will be return by PNNCAM_DATA_CALLBACK(V2), with the parameter 'bSnap' set to 'TRUE'
*/
NNCAM_API(HRESULT)  Nncam_Snap(HNncam h, unsigned nResolutionIndex);  /* still image snap */
NNCAM_API(HRESULT)  Nncam_SnapN(HNncam h, unsigned nResolutionIndex, unsigned nNumber);  /* multiple still image snap */
/*
    soft trigger:
    nNumber:    0xffff:     trigger continuously
                0:          cancel trigger
                others:     number of images to be triggered
*/
NNCAM_API(HRESULT)  Nncam_Trigger(HNncam h, unsigned short nNumber);

/*
    put_Size, put_eSize, can be used to set the video output resolution BEFORE Nncam_StartXXXX.
    put_Size use width and height parameters, put_eSize use the index parameter.
    for example, UCMOS03100KPA support the following resolutions:
            index 0:    2048,   1536
            index 1:    1024,   768
            index 2:    680,    510
    so, we can use put_Size(h, 1024, 768) or put_eSize(h, 1). Both have the same effect.
*/
NNCAM_API(HRESULT)  Nncam_put_Size(HNncam h, int nWidth, int nHeight);
NNCAM_API(HRESULT)  Nncam_get_Size(HNncam h, int* pWidth, int* pHeight);
NNCAM_API(HRESULT)  Nncam_put_eSize(HNncam h, unsigned nResolutionIndex);
NNCAM_API(HRESULT)  Nncam_get_eSize(HNncam h, unsigned* pnResolutionIndex);

/*
    final size after ROI, rotate, binning
*/
NNCAM_API(HRESULT)  Nncam_get_FinalSize(HNncam h, int* pWidth, int* pHeight);

NNCAM_API(HRESULT)  Nncam_get_ResolutionNumber(HNncam h);
NNCAM_API(HRESULT)  Nncam_get_Resolution(HNncam h, unsigned nResolutionIndex, int* pWidth, int* pHeight);
/*
    numerator/denominator, such as: 1/1, 1/2, 1/3
*/
NNCAM_API(HRESULT)  Nncam_get_ResolutionRatio(HNncam h, unsigned nResolutionIndex, int* pNumerator, int* pDenominator);
NNCAM_API(HRESULT)  Nncam_get_Field(HNncam h);

/*
see: http://www.fourcc.org
FourCC:
    MAKEFOURCC('G', 'B', 'R', 'G'), see http://www.siliconimaging.com/RGB%20Bayer.htm
    MAKEFOURCC('R', 'G', 'G', 'B')
    MAKEFOURCC('B', 'G', 'G', 'R')
    MAKEFOURCC('G', 'R', 'B', 'G')
    MAKEFOURCC('Y', 'Y', 'Y', 'Y'), monochromatic sensor
    MAKEFOURCC('Y', '4', '1', '1'), yuv411
    MAKEFOURCC('V', 'U', 'Y', 'Y'), yuv422
    MAKEFOURCC('U', 'Y', 'V', 'Y'), yuv422
    MAKEFOURCC('Y', '4', '4', '4'), yuv444
    MAKEFOURCC('R', 'G', 'B', '8'), RGB888

#ifndef MAKEFOURCC
#define MAKEFOURCC(a, b, c, d) ((unsigned)(unsigned char)(a) | ((unsigned)(unsigned char)(b) << 8) | ((unsigned)(unsigned char)(c) << 16) | ((unsigned)(unsigned char)(d) << 24))
#endif
*/
NNCAM_API(HRESULT)  Nncam_get_RawFormat(HNncam h, unsigned* nFourCC, unsigned* bitsperpixel);

/*
    ------------------------------------------------------------------|
    | Parameter               |   Range       |   Default             |
    |-----------------------------------------------------------------|
    | Auto Exposure Target    |   10~220      |   120                 |
    | Temp                    |   2000~15000  |   6503                |
    | Tint                    |   200~2500    |   1000                |
    | LevelRange              |   0~255       |   Low = 0, High = 255 |
    | Contrast                |   -100~100    |   0                   |
    | Hue                     |   -180~180    |   0                   |
    | Saturation              |   0~255       |   128                 |
    | Brightness              |   -64~64      |   0                   |
    | Gamma                   |   20~180      |   100                 |
    | WBGain                  |   -127~127    |   0                   |
    ------------------------------------------------------------------|
*/

#ifndef __NNCAM_CALLBACK_DEFINED__
#define __NNCAM_CALLBACK_DEFINED__
typedef void (__stdcall* PINNCAM_EXPOSURE_CALLBACK)(void* pCtx);                                     /* auto exposure */
typedef void (__stdcall* PINNCAM_WHITEBALANCE_CALLBACK)(const int aGain[3], void* pCtx);             /* once white balance, RGB Gain mode */
typedef void (__stdcall* PINNCAM_BLACKBALANCE_CALLBACK)(const unsigned short aSub[3], void* pCtx);   /* once black balance */
typedef void (__stdcall* PINNCAM_TEMPTINT_CALLBACK)(const int nTemp, const int nTint, void* pCtx);   /* once white balance, Temp/Tint Mode */
typedef void (__stdcall* PINNCAM_HISTOGRAM_CALLBACK)(const float aHistY[256], const float aHistR[256], const float aHistG[256], const float aHistB[256], void* pCtx);
typedef void (__stdcall* PINNCAM_CHROME_CALLBACK)(void* pCtx);
typedef void (__stdcall* PINNCAM_PROGRESS)(int percent, void* pCtx);
#endif

NNCAM_API(HRESULT)  Nncam_get_AutoExpoEnable(HNncam h, int* bAutoExposure);
NNCAM_API(HRESULT)  Nncam_put_AutoExpoEnable(HNncam h, int bAutoExposure);
NNCAM_API(HRESULT)  Nncam_get_AutoExpoTarget(HNncam h, unsigned short* Target);
NNCAM_API(HRESULT)  Nncam_put_AutoExpoTarget(HNncam h, unsigned short Target);

/*set the maximum/minimal auto exposure time and agin. The default maximum auto exposure time is 350ms */
NNCAM_API(HRESULT)  Nncam_put_MaxAutoExpoTimeAGain(HNncam h, unsigned maxTime, unsigned short maxAGain);
NNCAM_API(HRESULT)  Nncam_get_MaxAutoExpoTimeAGain(HNncam h, unsigned* maxTime, unsigned short* maxAGain);
NNCAM_API(HRESULT)  Nncam_put_MinAutoExpoTimeAGain(HNncam h, unsigned minTime, unsigned short minAGain);
NNCAM_API(HRESULT)  Nncam_get_MinAutoExpoTimeAGain(HNncam h, unsigned* minTime, unsigned short* minAGain);

NNCAM_API(HRESULT)  Nncam_get_ExpoTime(HNncam h, unsigned* Time); /* in microseconds */
NNCAM_API(HRESULT)  Nncam_put_ExpoTime(HNncam h, unsigned Time); /* in microseconds */
NNCAM_API(HRESULT)  Nncam_get_RealExpoTime(HNncam h, unsigned* Time); /* in microseconds, based on 50HZ/60HZ/DC */
NNCAM_API(HRESULT)  Nncam_get_ExpTimeRange(HNncam h, unsigned* nMin, unsigned* nMax, unsigned* nDef);

NNCAM_API(HRESULT)  Nncam_get_ExpoAGain(HNncam h, unsigned short* AGain); /* percent, such as 300 */
NNCAM_API(HRESULT)  Nncam_put_ExpoAGain(HNncam h, unsigned short AGain); /* percent */
NNCAM_API(HRESULT)  Nncam_get_ExpoAGainRange(HNncam h, unsigned short* nMin, unsigned short* nMax, unsigned short* nDef);

/* Auto White Balance "Once", Temp/Tint Mode */
NNCAM_API(HRESULT)  Nncam_AwbOnce(HNncam h, PINNCAM_TEMPTINT_CALLBACK fnTTProc, void* pTTCtx); /* auto white balance "once". This function must be called AFTER Nncam_StartXXXX */

/* Auto White Balance, RGB Gain Mode */
NNCAM_API(HRESULT)  Nncam_AwbInit(HNncam h, PINNCAM_WHITEBALANCE_CALLBACK fnWBProc, void* pWBCtx);

/* White Balance, Temp/Tint mode */
NNCAM_API(HRESULT)  Nncam_put_TempTint(HNncam h, int nTemp, int nTint);
NNCAM_API(HRESULT)  Nncam_get_TempTint(HNncam h, int* nTemp, int* nTint);

/* White Balance, RGB Gain mode */
NNCAM_API(HRESULT)  Nncam_put_WhiteBalanceGain(HNncam h, int aGain[3]);
NNCAM_API(HRESULT)  Nncam_get_WhiteBalanceGain(HNncam h, int aGain[3]);

/* Black Balance */
NNCAM_API(HRESULT)  Nncam_AbbOnce(HNncam h, PINNCAM_BLACKBALANCE_CALLBACK fnBBProc, void* pBBCtx); /* auto black balance "once". This function must be called AFTER Nncam_StartXXXX */
NNCAM_API(HRESULT)  Nncam_put_BlackBalance(HNncam h, unsigned short aSub[3]);
NNCAM_API(HRESULT)  Nncam_get_BlackBalance(HNncam h, unsigned short aSub[3]);

/* Flat Field Correction */
NNCAM_API(HRESULT)  Nncam_FfcOnce(HNncam h);
#ifdef _WIN32
NNCAM_API(HRESULT)  Nncam_FfcExport(HNncam h, const wchar_t* filepath);
NNCAM_API(HRESULT)  Nncam_FfcImport(HNncam h, const wchar_t* filepath);
#else
NNCAM_API(HRESULT)  Nncam_FfcExport(HNncam h, const char* filepath);
NNCAM_API(HRESULT)  Nncam_FfcImport(HNncam h, const char* filepath);
#endif

/* Dark Field Correction */
NNCAM_API(HRESULT)  Nncam_DfcOnce(HNncam h);

#ifdef _WIN32
NNCAM_API(HRESULT)  Nncam_DfcExport(HNncam h, const wchar_t* filepath);
NNCAM_API(HRESULT)  Nncam_DfcImport(HNncam h, const wchar_t* filepath);
#else
NNCAM_API(HRESULT)  Nncam_DfcExport(HNncam h, const char* filepath);
NNCAM_API(HRESULT)  Nncam_DfcImport(HNncam h, const char* filepath);
#endif

NNCAM_API(HRESULT)  Nncam_put_Hue(HNncam h, int Hue);
NNCAM_API(HRESULT)  Nncam_get_Hue(HNncam h, int* Hue);
NNCAM_API(HRESULT)  Nncam_put_Saturation(HNncam h, int Saturation);
NNCAM_API(HRESULT)  Nncam_get_Saturation(HNncam h, int* Saturation);
NNCAM_API(HRESULT)  Nncam_put_Brightness(HNncam h, int Brightness);
NNCAM_API(HRESULT)  Nncam_get_Brightness(HNncam h, int* Brightness);
NNCAM_API(HRESULT)  Nncam_get_Contrast(HNncam h, int* Contrast);
NNCAM_API(HRESULT)  Nncam_put_Contrast(HNncam h, int Contrast);
NNCAM_API(HRESULT)  Nncam_get_Gamma(HNncam h, int* Gamma); /* percent */
NNCAM_API(HRESULT)  Nncam_put_Gamma(HNncam h, int Gamma);  /* percent */

NNCAM_API(HRESULT)  Nncam_get_Chrome(HNncam h, int* bChrome);  /* monochromatic mode */
NNCAM_API(HRESULT)  Nncam_put_Chrome(HNncam h, int bChrome);

NNCAM_API(HRESULT)  Nncam_get_VFlip(HNncam h, int* bVFlip);  /* vertical flip */
NNCAM_API(HRESULT)  Nncam_put_VFlip(HNncam h, int bVFlip);
NNCAM_API(HRESULT)  Nncam_get_HFlip(HNncam h, int* bHFlip);
NNCAM_API(HRESULT)  Nncam_put_HFlip(HNncam h, int bHFlip); /* horizontal flip */

NNCAM_API(HRESULT)  Nncam_get_Negative(HNncam h, int* bNegative);  /* negative film */
NNCAM_API(HRESULT)  Nncam_put_Negative(HNncam h, int bNegative);

NNCAM_API(HRESULT)  Nncam_put_Speed(HNncam h, unsigned short nSpeed);
NNCAM_API(HRESULT)  Nncam_get_Speed(HNncam h, unsigned short* pSpeed);
NNCAM_API(HRESULT)  Nncam_get_MaxSpeed(HNncam h); /* get the maximum speed, see "Frame Speed Level", the speed range = [0, max], closed interval */

NNCAM_API(HRESULT)  Nncam_get_FanMaxSpeed(HNncam h); /* get the maximum fan speed, the fan speed range = [0, max], closed interval */

NNCAM_API(HRESULT)  Nncam_get_MaxBitDepth(HNncam h); /* get the max bit depth of this camera, such as 8, 10, 12, 14, 16 */

/* power supply of lighting:
        0 -> 60HZ AC
        1 -> 50Hz AC
        2 -> DC
*/
NNCAM_API(HRESULT)  Nncam_put_HZ(HNncam h, int nHZ);
NNCAM_API(HRESULT)  Nncam_get_HZ(HNncam h, int* nHZ);

NNCAM_API(HRESULT)  Nncam_put_Mode(HNncam h, int bSkip); /* skip or bin */
NNCAM_API(HRESULT)  Nncam_get_Mode(HNncam h, int* bSkip); /* If the model don't support bin/skip mode, return E_NOTIMPL */

NNCAM_API(HRESULT)  Nncam_put_AWBAuxRect(HNncam h, const RECT* pAuxRect); /* auto white balance ROI */
NNCAM_API(HRESULT)  Nncam_get_AWBAuxRect(HNncam h, RECT* pAuxRect);
NNCAM_API(HRESULT)  Nncam_put_AEAuxRect(HNncam h, const RECT* pAuxRect);  /* auto exposure ROI */
NNCAM_API(HRESULT)  Nncam_get_AEAuxRect(HNncam h, RECT* pAuxRect);

NNCAM_API(HRESULT)  Nncam_put_ABBAuxRect(HNncam h, const RECT* pAuxRect); /* auto black balance ROI */
NNCAM_API(HRESULT)  Nncam_get_ABBAuxRect(HNncam h, RECT* pAuxRect);

/*
    S_FALSE:    color mode
    S_OK:       mono mode, such as EXCCD00300KMA and UHCCD01400KMA
*/
NNCAM_API(HRESULT)  Nncam_get_MonoMode(HNncam h);

NNCAM_API(HRESULT)  Nncam_get_StillResolutionNumber(HNncam h);
NNCAM_API(HRESULT)  Nncam_get_StillResolution(HNncam h, unsigned nResolutionIndex, int* pWidth, int* pHeight);

/*  0: stop grab frame when frame buffer deque is full, until the frames in the queue are pulled away and the queue is not full
    1: realtime
          use minimum frame buffer. When new frame arrive, drop all the pending frame regardless of whether the frame buffer is full.
          If DDR present, also limit the DDR frame buffer to only one frame.
    2: soft realtime
          Drop the oldest frame when the queue is full and then enqueue the new frame
    default: 0
*/
NNCAM_API(HRESULT)  Nncam_put_RealTime(HNncam h, int val);
NNCAM_API(HRESULT)  Nncam_get_RealTime(HNncam h, int* val);

/* discard the current internal frame cache.
    If DDR present, also discard the frames in the DDR.
    Nncam_Flush is obsolete, it's a synonyms for Nncam_Flush(h, NNCAM_OPTION_FLUSH, 3)
*/
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_Flush(HNncam h);

/* get the temperature of the sensor, in 0.1 degrees Celsius (32 means 3.2 degrees Celsius, -35 means -3.5 degree Celsius)
    return E_NOTIMPL if not supported
*/
NNCAM_API(HRESULT)  Nncam_get_Temperature(HNncam h, short* pTemperature);

/* set the target temperature of the sensor or TEC, in 0.1 degrees Celsius (32 means 3.2 degrees Celsius, -35 means -3.5 degree Celsius)
    return E_NOTIMPL if not supported
*/
NNCAM_API(HRESULT)  Nncam_put_Temperature(HNncam h, short nTemperature);

/*
    get the revision
*/
NNCAM_API(HRESULT)  Nncam_get_Revision(HNncam h, unsigned short* pRevision);

/*
    get the serial number which is always 32 chars which is zero-terminated such as "TP110826145730ABCD1234FEDC56787"
*/
NNCAM_API(HRESULT)  Nncam_get_SerialNumber(HNncam h, char sn[32]);

/*
    get the camera firmware version, such as: 3.2.1.20140922
*/
NNCAM_API(HRESULT)  Nncam_get_FwVersion(HNncam h, char fwver[16]);

/*
    get the camera hardware version, such as: 3.12
*/
NNCAM_API(HRESULT)  Nncam_get_HwVersion(HNncam h, char hwver[16]);

/*
    get the production date, such as: 20150327, YYYYMMDD, (YYYY: year, MM: month, DD: day)
*/
NNCAM_API(HRESULT)  Nncam_get_ProductionDate(HNncam h, char pdate[10]);

/*
    get the FPGA version, such as: 1.13
*/
NNCAM_API(HRESULT)  Nncam_get_FpgaVersion(HNncam h, char fpgaver[16]);

/*
    get the sensor pixel size, such as: 2.4um
*/
NNCAM_API(HRESULT)  Nncam_get_PixelSize(HNncam h, unsigned nResolutionIndex, float* x, float* y);

/* software level range */
NNCAM_API(HRESULT)  Nncam_put_LevelRange(HNncam h, unsigned short aLow[4], unsigned short aHigh[4]);
NNCAM_API(HRESULT)  Nncam_get_LevelRange(HNncam h, unsigned short aLow[4], unsigned short aHigh[4]);

/* hardware level range mode */
#define NNCAM_LEVELRANGE_MANUAL       0x0000  /* manual */
#define NNCAM_LEVELRANGE_ONCE         0x0001  /* once */
#define NNCAM_LEVELRANGE_CONTINUE     0x0002  /* continue */
#define NNCAM_LEVELRANGE_ROI          0xffff  /* update roi rect only */
NNCAM_API(HRESULT)  Nncam_put_LevelRangeV2(HNncam h, unsigned short mode, const RECT* pRoiRect, unsigned short aLow[4], unsigned short aHigh[4]);
NNCAM_API(HRESULT)  Nncam_get_LevelRangeV2(HNncam h, unsigned short* pMode, RECT* pRoiRect, unsigned short aLow[4], unsigned short aHigh[4]);

/*
    The following functions must be called AFTER Nncam_StartPushMode or Nncam_StartPullModeWithWndMsg or Nncam_StartPullModeWithCallback
*/
NNCAM_API(HRESULT)  Nncam_LevelRangeAuto(HNncam h);  /* software level range */
NNCAM_API(HRESULT)  Nncam_GetHistogram(HNncam h, PINNCAM_HISTOGRAM_CALLBACK fnHistogramProc, void* pHistogramCtx);

/* led state:
    iLed: Led index, (0, 1, 2, ...)
    iState: 1 -> Ever bright; 2 -> Flashing; other -> Off
    iPeriod: Flashing Period (>= 500ms)
*/
NNCAM_API(HRESULT)  Nncam_put_LEDState(HNncam h, unsigned short iLed, unsigned short iState, unsigned short iPeriod);

NNCAM_API(HRESULT)  Nncam_write_EEPROM(HNncam h, unsigned addr, const unsigned char* pBuffer, unsigned nBufferLen);
NNCAM_API(HRESULT)  Nncam_read_EEPROM(HNncam h, unsigned addr, unsigned char* pBuffer, unsigned nBufferLen);

NNCAM_API(HRESULT)  Nncam_read_Pipe(HNncam h, unsigned pipeNum, void* pBuffer, unsigned nBufferLen);
NNCAM_API(HRESULT)  Nncam_write_Pipe(HNncam h, unsigned pipeNum, const void* pBuffer, unsigned nBufferLen);
NNCAM_API(HRESULT)  Nncam_feed_Pipe(HNncam h, unsigned pipeNum);

/* TEC target range */
#define NNCAM_TEC_TARGET_MIN               (-300)     /* -30.0 degrees Celsius */
#define NNCAM_TEC_TARGET_DEF               0          /* 0.0 degrees Celsius */
#define NNCAM_TEC_TARGET_MAX               300        /* 30.0 degrees Celsius */
                                             
#define NNCAM_OPTION_NOFRAME_TIMEOUT       0x01       /* no frame timeout: 1 = enable; 0 = disable. default: disable */
#define NNCAM_OPTION_THREAD_PRIORITY       0x02       /* set the priority of the internal thread which grab data from the usb device.
                                                            Win: iValue: 0 = THREAD_PRIORITY_NORMAL; 1 = THREAD_PRIORITY_ABOVE_NORMAL; 2 = THREAD_PRIORITY_HIGHEST; 3 = THREAD_PRIORITY_TIME_CRITICAL; default: 1; see: msdn SetThreadPriority
                                                            Linux & macOS: The high 16 bits for the scheduling policy, and the low 16 bits for the priority
                                                        */
#define NNCAM_OPTION_PROCESSMODE           0x03       /* obsolete & useless. 0 = better image quality, more cpu usage. this is the default value; 1 = lower image quality, less cpu usage */
#define NNCAM_OPTION_RAW                   0x04       /* raw data mode, read the sensor "raw" data. This can be set only BEFORE Nncam_StartXXX(). 0 = rgb, 1 = raw, default value: 0 */
#define NNCAM_OPTION_HISTOGRAM             0x05       /* 0 = only one, 1 = continue mode */
#define NNCAM_OPTION_BITDEPTH              0x06       /* 0 = 8 bits mode, 1 = 16 bits mode, subset of NNCAM_OPTION_PIXEL_FORMAT */
#define NNCAM_OPTION_FAN                   0x07       /* 0 = turn off the cooling fan, [1, max] = fan speed */
#define NNCAM_OPTION_TEC                   0x08       /* 0 = turn off the thermoelectric cooler, 1 = turn on the thermoelectric cooler */
#define NNCAM_OPTION_LINEAR                0x09       /* 0 = turn off the builtin linear tone mapping, 1 = turn on the builtin linear tone mapping, default value: 1 */
#define NNCAM_OPTION_CURVE                 0x0a       /* 0 = turn off the builtin curve tone mapping, 1 = turn on the builtin polynomial curve tone mapping, 2 = logarithmic curve tone mapping, default value: 2 */
#define NNCAM_OPTION_TRIGGER               0x0b       /* 0 = video mode, 1 = software or simulated trigger mode, 2 = external trigger mode, 3 = external + software trigger, default value = 0 */
#define NNCAM_OPTION_RGB                   0x0c       /* 0 => RGB24; 1 => enable RGB48 format when bitdepth > 8; 2 => RGB32; 3 => 8 Bits Gray (only for mono camera); 4 => 16 Bits Gray (only for mono camera when bitdepth > 8) */
#define NNCAM_OPTION_COLORMATIX            0x0d       /* enable or disable the builtin color matrix, default value: 1 */
#define NNCAM_OPTION_WBGAIN                0x0e       /* enable or disable the builtin white balance gain, default value: 1 */
#define NNCAM_OPTION_TECTARGET             0x0f       /* get or set the target temperature of the thermoelectric cooler, in 0.1 degree Celsius. For example, 125 means 12.5 degree Celsius, -35 means -3.5 degree Celsius */
#define NNCAM_OPTION_AUTOEXP_POLICY        0x10       /* auto exposure policy:
                                                            0: Exposure Only
                                                            1: Exposure Preferred
                                                            2: Gain Only
                                                            3: Gain Preferred
                                                            default value: 1
                                                        */
#define NNCAM_OPTION_FRAMERATE             0x11       /* limit the frame rate, range=[0, 63], the default value 0 means no limit */
#define NNCAM_OPTION_DEMOSAIC              0x12       /* demosaic method for both video and still image: BILINEAR = 0, VNG(Variable Number of Gradients interpolation) = 1, PPG(Patterned Pixel Grouping interpolation) = 2, AHD(Adaptive Homogeneity-Directed interpolation) = 3, see https://en.wikipedia.org/wiki/Demosaicing, default value: 0 */
#define NNCAM_OPTION_DEMOSAIC_VIDEO        0x13       /* demosaic method for video */
#define NNCAM_OPTION_DEMOSAIC_STILL        0x14       /* demosaic method for still image */
#define NNCAM_OPTION_BLACKLEVEL            0x15       /* black level */
#define NNCAM_OPTION_MULTITHREAD           0x16       /* multithread image processing */
#define NNCAM_OPTION_BINNING               0x17       /* binning, 0x01 (no binning), 0x02 (add, 2*2), 0x03 (add, 3*3), 0x04 (add, 4*4), 0x05 (add, 5*5), 0x06 (add, 6*6), 0x07 (add, 7*7), 0x08 (add, 8*8), 0x82 (average, 2*2), 0x83 (average, 3*3), 0x84 (average, 4*4), 0x85 (average, 5*5), 0x86 (average, 6*6), 0x87 (average, 7*7), 0x88 (average, 8*8). The final image size is rounded down to an even number, such as 640/3 to get 212 */
#define NNCAM_OPTION_ROTATE                0x18       /* rotate clockwise: 0, 90, 180, 270 */
#define NNCAM_OPTION_CG                    0x19       /* Conversion Gain: 0 = LCG, 1 = HCG, 2 = HDR */
#define NNCAM_OPTION_PIXEL_FORMAT          0x1a       /* pixel format, NNCAM_PIXELFORMAT_xxxx */
#define NNCAM_OPTION_FFC                   0x1b       /* flat field correction
                                                            set:
                                                                 0: disable
                                                                 1: enable
                                                                -1: reset
                                                                (0xff000000 | n): set the average number to n, [1~255]
                                                            get:
                                                                 (val & 0xff): 0 -> disable, 1 -> enable, 2 -> inited
                                                                 ((val & 0xff00) >> 8): sequence
                                                                 ((val & 0xff0000) >> 8): average number
                                                        */
#define NNCAM_OPTION_DDR_DEPTH             0x1c       /* the number of the frames that DDR can cache
                                                                1: DDR cache only one frame
                                                                0: Auto:
                                                                        ->one for video mode when auto exposure is enabled
                                                                        ->full capacity for others
                                                               -1: DDR can cache frames to full capacity
                                                        */
#define NNCAM_OPTION_DFC                   0x1d       /* dark field correction
                                                            set:
                                                                0: disable
                                                                1: enable
                                                               -1: reset
                                                                (0xff000000 | n): set the average number to n, [1~255]
                                                            get:
                                                                (val & 0xff): 0 -> disable, 1 -> enable, 2 -> inited
                                                                ((val & 0xff00) >> 8): sequence
                                                                ((val & 0xff0000) >> 8): average number
                                                        */
#define NNCAM_OPTION_SHARPENING            0x1e       /* Sharpening: (threshold << 24) | (radius << 16) | strength)
                                                            strength: [0, 500], default: 0 (disable)
                                                            radius: [1, 10]
                                                            threshold: [0, 255]
                                                        */
#define NNCAM_OPTION_FACTORY               0x1f       /* restore the factory settings */
#define NNCAM_OPTION_TEC_VOLTAGE           0x20       /* get the current TEC voltage in 0.1V, 59 mean 5.9V; readonly */
#define NNCAM_OPTION_TEC_VOLTAGE_MAX       0x21       /* get the TEC maximum voltage in 0.1V; readonly */
#define NNCAM_OPTION_DEVICE_RESET          0x22       /* reset usb device, simulate a replug */
#define NNCAM_OPTION_UPSIDE_DOWN           0x23       /* upsize down:
                                                            1: yes
                                                            0: no
                                                            default: 1 (win), 0 (linux/macos)
                                                        */
#define NNCAM_OPTION_AFPOSITION            0x24       /* auto focus sensor board positon */
#define NNCAM_OPTION_AFMODE                0x25       /* auto focus mode (0:manul focus; 1:auto focus; 2:once focus; 3:conjugate calibration) */
#define NNCAM_OPTION_AFZONE                0x26       /* auto focus zone */
#define NNCAM_OPTION_AFFEEDBACK            0x27       /* auto focus information feedback; 0:unknown; 1:focused; 2:focusing; 3:defocus; 4:up; 5:down */
#define NNCAM_OPTION_TESTPATTERN           0x28       /* test pattern:
                                                           0: TestPattern Off
                                                           3: monochrome diagonal stripes
                                                           5: monochrome vertical stripes
                                                           7: monochrome horizontal stripes
                                                           9: chromatic diagonal stripes
                                                        */
#define NNCAM_OPTION_AUTOEXP_THRESHOLD     0x29       /* threshold of auto exposure, default value: 5, range = [2, 15] */
#define NNCAM_OPTION_BYTEORDER             0x2a       /* Byte order, BGR or RGB: 0->RGB, 1->BGR, default value: 1(Win), 0(macOS, Linux, Android) */
#define NNCAM_OPTION_NOPACKET_TIMEOUT      0x2b       /* no packet timeout: 0 = disable, positive value = timeout milliseconds. default: disable */
#define NNCAM_OPTION_MAX_PRECISE_FRAMERATE 0x2c       /* precise frame rate maximum value in 0.1 fps, such as 115 means 11.5 fps */
#define NNCAM_OPTION_PRECISE_FRAMERATE     0x2d       /* precise frame rate current value in 0.1 fps */
#define NNCAM_OPTION_BANDWIDTH             0x2e       /* bandwidth, [1-100]% */
#define NNCAM_OPTION_RELOAD                0x2f       /* reload the last frame in trigger mode */
#define NNCAM_OPTION_CALLBACK_THREAD       0x30       /* dedicated thread for callback */
#define NNCAM_OPTION_FRAME_DEQUE_LENGTH    0x31       /* frame buffer deque length, range: [2, 1024], default: 3 */
#define NNCAM_OPTION_MIN_PRECISE_FRAMERATE 0x32       /* precise frame rate minimum value in 0.1 fps, such as 15 means 1.5 fps */
#define NNCAM_OPTION_SEQUENCER_ONOFF       0x33       /* sequencer trigger: on/off */
#define NNCAM_OPTION_SEQUENCER_NUMBER      0x34       /* sequencer trigger: number, range = [1, 255] */
#define NNCAM_OPTION_SEQUENCER_EXPOTIME    0x01000000 /* sequencer trigger: exposure time, iOption = NNCAM_OPTION_SEQUENCER_EXPOTIME | index, iValue = exposure time
                                                            For example, to set the exposure time of the third group to 50ms, call:
                                                               Nncam_put_Option(NNCAM_OPTION_SEQUENCER_EXPOTIME | 3, 50000)
                                                        */
#define NNCAM_OPTION_SEQUENCER_EXPOGAIN    0x02000000 /* sequencer trigger: exposure gain, iOption = NNCAM_OPTION_SEQUENCER_EXPOGAIN | index, iValue = gain */
#define NNCAM_OPTION_DENOISE               0x35       /* denoise, strength range: [0, 100], 0 means disable */
#define NNCAM_OPTION_HEAT_MAX              0x36       /* get maximum level: heat to prevent fogging up */
#define NNCAM_OPTION_HEAT                  0x37       /* heat to prevent fogging up */
#define NNCAM_OPTION_LOW_NOISE             0x38       /* low noise mode (Higher signal noise ratio, lower frame rate): 1 => enable */
#define NNCAM_OPTION_POWER                 0x39       /* get power consumption, unit: milliwatt */
#define NNCAM_OPTION_GLOBAL_RESET_MODE     0x3a       /* global reset mode */
#define NNCAM_OPTION_OPEN_USB_ERRORCODE    0x3b       /* open usb error code */
#define NNCAM_OPTION_LINUX_USB_ZEROCOPY    0x3c       /* global option for linux platform:
                                                             enable or disable usb zerocopy (helps to reduce memory copy and improve efficiency. Requires kernel version >= 4.6 and hardware platform support)
                                                             if the image is wrong, this indicates that the hardware platform does not support this feature, please disable it when the program starts:
                                                               Nncam_put_Option((this is a global option, the camera handle parameter is not required, use nullptr), NNCAM_OPTION_LINUX_USB_ZEROCOPY, 0)
                                                             default value:
                                                               disable(0): android or arm32
                                                               enable(1):  others
                                                        */
#define NNCAM_OPTION_FLUSH                 0x3d       /* 1 = hard flush, discard frames cached by camera DDR (if any)
                                                           2 = soft flush, discard frames cached by nncam.dll (if any)
                                                           3 = both flush
                                                           Nncam_Flush means 'both flush'
                                                        */
#define NNCAM_OPTION_NUMBER_DROP_FRAME     0x3e       /* get the number of frames that have been grabbed from the USB but dropped by the software */
#define NNCAM_OPTION_DUMP_CFG              0x3f       /* explicitly dump configuration to ini, json, or EEPROM. when camera is closed, it will dump configuration automatically */

/* pixel format */
#define NNCAM_PIXELFORMAT_RAW8             0x00
#define NNCAM_PIXELFORMAT_RAW10            0x01
#define NNCAM_PIXELFORMAT_RAW12            0x02
#define NNCAM_PIXELFORMAT_RAW14            0x03
#define NNCAM_PIXELFORMAT_RAW16            0x04
#define NNCAM_PIXELFORMAT_YUV411           0x05
#define NNCAM_PIXELFORMAT_VUYY             0x06
#define NNCAM_PIXELFORMAT_YUV444           0x07
#define NNCAM_PIXELFORMAT_RGB888           0x08
#define NNCAM_PIXELFORMAT_GMCY8            0x09   /* map to RGGB 8 bits */
#define NNCAM_PIXELFORMAT_GMCY12           0x0a   /* map to RGGB 12 bits */
#define NNCAM_PIXELFORMAT_UYVY             0x0b

NNCAM_API(HRESULT)  Nncam_put_Option(HNncam h, unsigned iOption, int iValue);
NNCAM_API(HRESULT)  Nncam_get_Option(HNncam h, unsigned iOption, int* piValue);

/*
    xOffset, yOffset, xWidth, yHeight: must be even numbers
*/
NNCAM_API(HRESULT)  Nncam_put_Roi(HNncam h, unsigned xOffset, unsigned yOffset, unsigned xWidth, unsigned yHeight);
NNCAM_API(HRESULT)  Nncam_get_Roi(HNncam h, unsigned* pxOffset, unsigned* pyOffset, unsigned* pxWidth, unsigned* pyHeight);

/*  simulate replug:
    return > 0, the number of device has been replug
    return = 0, no device found
    return E_ACCESSDENIED if without UAC Administrator privileges
    for each device found, it will take about 3 seconds
*/
#ifdef _WIN32
NNCAM_API(HRESULT) Nncam_Replug(const wchar_t* id);
#else
NNCAM_API(HRESULT) Nncam_Replug(const char* id);
#endif

#ifndef __NNCAMAFPARAM_DEFINED__
#define __NNCAMAFPARAM_DEFINED__
typedef struct {
    int imax;    /* maximum auto focus sensor board positon */
    int imin;    /* minimum auto focus sensor board positon */
    int idef;    /* conjugate calibration positon */
    int imaxabs; /* maximum absolute auto focus sensor board positon, micrometer */
    int iminabs; /* maximum absolute auto focus sensor board positon, micrometer */
    int zoneh;   /* zone horizontal */
    int zonev;   /* zone vertical */
}NncamAfParam;
#endif

NNCAM_API(HRESULT)  Nncam_get_AfParam(HNncam h, NncamAfParam* pAfParam);

#define NNCAM_IOCONTROLTYPE_GET_SUPPORTEDMODE           0x01 /* 0x01->Input, 0x02->Output, (0x01 | 0x02)->support both Input and Output */
#define NNCAM_IOCONTROLTYPE_GET_GPIODIR                 0x03 /* 0x00->Input, 0x01->Output */
#define NNCAM_IOCONTROLTYPE_SET_GPIODIR                 0x04
#define NNCAM_IOCONTROLTYPE_GET_FORMAT                  0x05 /*
                                                                   0x00-> not connected
                                                                   0x01-> Tri-state: Tri-state mode (Not driven)
                                                                   0x02-> TTL: TTL level signals
                                                                   0x03-> LVDS: LVDS level signals
                                                                   0x04-> RS422: RS422 level signals
                                                                   0x05-> Opto-coupled
                                                               */
#define NNCAM_IOCONTROLTYPE_SET_FORMAT                  0x06
#define NNCAM_IOCONTROLTYPE_GET_OUTPUTINVERTER          0x07 /* boolean, only support output signal */
#define NNCAM_IOCONTROLTYPE_SET_OUTPUTINVERTER          0x08
#define NNCAM_IOCONTROLTYPE_GET_INPUTACTIVATION         0x09 /* 0x00->Positive, 0x01->Negative */
#define NNCAM_IOCONTROLTYPE_SET_INPUTACTIVATION         0x0a
#define NNCAM_IOCONTROLTYPE_GET_DEBOUNCERTIME           0x0b /* debouncer time in microseconds, [0, 20000] */
#define NNCAM_IOCONTROLTYPE_SET_DEBOUNCERTIME           0x0c
#define NNCAM_IOCONTROLTYPE_GET_TRIGGERSOURCE           0x0d /*
                                                                  0x00-> Opto-isolated input
                                                                  0x01-> GPIO0
                                                                  0x02-> GPIO1
                                                                  0x03-> Counter
                                                                  0x04-> PWM
                                                                  0x05-> Software
                                                               */
#define NNCAM_IOCONTROLTYPE_SET_TRIGGERSOURCE           0x0e
#define NNCAM_IOCONTROLTYPE_GET_TRIGGERDELAY            0x0f /* Trigger delay time in microseconds, [0, 5000000] */
#define NNCAM_IOCONTROLTYPE_SET_TRIGGERDELAY            0x10
#define NNCAM_IOCONTROLTYPE_GET_BURSTCOUNTER            0x11 /* Burst Counter: 1, 2, 3 ... 1023 */
#define NNCAM_IOCONTROLTYPE_SET_BURSTCOUNTER            0x12
#define NNCAM_IOCONTROLTYPE_GET_COUNTERSOURCE           0x13 /* 0x00-> Opto-isolated input, 0x01-> GPIO0, 0x02-> GPIO1 */
#define NNCAM_IOCONTROLTYPE_SET_COUNTERSOURCE           0x14
#define NNCAM_IOCONTROLTYPE_GET_COUNTERVALUE            0x15 /* Counter Value: 1, 2, 3 ... 1023 */
#define NNCAM_IOCONTROLTYPE_SET_COUNTERVALUE            0x16
#define NNCAM_IOCONTROLTYPE_SET_RESETCOUNTER            0x18
#define NNCAM_IOCONTROLTYPE_GET_PWM_FREQ                0x19
#define NNCAM_IOCONTROLTYPE_SET_PWM_FREQ                0x1a
#define NNCAM_IOCONTROLTYPE_GET_PWM_DUTYRATIO           0x1b
#define NNCAM_IOCONTROLTYPE_SET_PWM_DUTYRATIO           0x1c
#define NNCAM_IOCONTROLTYPE_GET_PWMSOURCE               0x1d /* 0x00-> Opto-isolated input, 0x01-> GPIO0, 0x02-> GPIO1 */
#define NNCAM_IOCONTROLTYPE_SET_PWMSOURCE               0x1e
#define NNCAM_IOCONTROLTYPE_GET_OUTPUTMODE              0x1f /*
                                                                  0x00-> Frame Trigger Wait
                                                                  0x01-> Exposure Active
                                                                  0x02-> Strobe
                                                                  0x03-> User output
                                                               */
#define NNCAM_IOCONTROLTYPE_SET_OUTPUTMODE              0x20
#define NNCAM_IOCONTROLTYPE_GET_STROBEDELAYMODE         0x21 /* boolean, 0-> pre-delay, 1-> delay; compared to exposure active signal */
#define NNCAM_IOCONTROLTYPE_SET_STROBEDELAYMODE         0x22
#define NNCAM_IOCONTROLTYPE_GET_STROBEDELAYTIME         0x23 /* Strobe delay or pre-delay time in microseconds, [0, 5000000] */
#define NNCAM_IOCONTROLTYPE_SET_STROBEDELAYTIME         0x24
#define NNCAM_IOCONTROLTYPE_GET_STROBEDURATION          0x25 /* Strobe duration time in microseconds, [0, 5000000] */
#define NNCAM_IOCONTROLTYPE_SET_STROBEDURATION          0x26
#define NNCAM_IOCONTROLTYPE_GET_USERVALUE               0x27 /* 
                                                                  bit0-> Opto-isolated output
                                                                  bit1-> GPIO0 output
                                                                  bit2-> GPIO1 output
                                                               */
#define NNCAM_IOCONTROLTYPE_SET_USERVALUE               0x28
#define NNCAM_IOCONTROLTYPE_GET_UART_ENABLE             0x29 /* enable: 1-> on; 0-> off */
#define NNCAM_IOCONTROLTYPE_SET_UART_ENABLE             0x2a
#define NNCAM_IOCONTROLTYPE_GET_UART_BAUDRATE           0x2b /* baud rate: 0-> 9600; 1-> 19200; 2-> 38400; 3-> 57600; 4-> 115200 */
#define NNCAM_IOCONTROLTYPE_SET_UART_BAUDRATE           0x2c
#define NNCAM_IOCONTROLTYPE_GET_UART_LINEMODE           0x2d /* line mode: 0-> TX(GPIO_0)/RX(GPIO_1); 1-> TX(GPIO_1)/RX(GPIO_0) */
#define NNCAM_IOCONTROLTYPE_SET_UART_LINEMODE           0x2e

#define NNCAM_IOCONTROL_DELAYTIME_MAX                   (5 * 1000 * 1000)

NNCAM_API(HRESULT)  Nncam_IoControl(HNncam h, unsigned index, unsigned nType, int outVal, int* inVal);

NNCAM_API(HRESULT)  Nncam_write_UART(HNncam h, const unsigned char* pData, unsigned nDataLen);
NNCAM_API(HRESULT)  Nncam_read_UART(HNncam h, unsigned char* pBuffer, unsigned nBufferLen);

/* firmware update:
    camId: camera ID
    filePath: ufw file full path
    pFun, pCtx: progress percent callback
Please do not unplug the camera or lost power during the upgrade process, this is very very important.
Once an unplugging or power outage occurs during the upgrade process, the camera will no longer be available and can only be returned to the factory for repair.
*/
#ifdef _WIN32
NNCAM_API(HRESULT)  Nncam_Update(const wchar_t* camId, const wchar_t* filePath, PINNCAM_PROGRESS pFun, void* pCtx);
#else
NNCAM_API(HRESULT)  Nncam_Update(const char* camId, const char* filePath, PINNCAM_PROGRESS pFun, void* pCtx);
#endif

NNCAM_API(HRESULT)  Nncam_put_Linear(HNncam h, const unsigned char* v8, const unsigned short* v16);
NNCAM_API(HRESULT)  Nncam_put_Curve(HNncam h, const unsigned char* v8, const unsigned short* v16);
NNCAM_API(HRESULT)  Nncam_put_ColorMatrix(HNncam h, const double v[9]);
NNCAM_API(HRESULT)  Nncam_put_InitWBGain(HNncam h, const unsigned short v[3]);

/*
    get the frame rate: framerate (fps) = Frame * 1000.0 / nTime
*/
NNCAM_API(HRESULT)  Nncam_get_FrameRate(HNncam h, unsigned* nFrame, unsigned* nTime, unsigned* nTotalFrame);

/* astronomy: for ST4 guide, please see: ASCOM Platform Help ICameraV2.
    nDirect: 0 = North, 1 = South, 2 = East, 3 = West, 4 = Stop
    nDuration: in milliseconds
*/
NNCAM_API(HRESULT)  Nncam_ST4PlusGuide(HNncam h, unsigned nDirect, unsigned nDuration);

/* S_OK: ST4 pulse guiding
   S_FALSE: ST4 not pulse guiding
*/
NNCAM_API(HRESULT)  Nncam_ST4PlusGuideState(HNncam h);

/*
    calculate the clarity factor:
    pImageData: pointer to the image data
    bits: 8(Grey), 24 (RGB24), 32(RGB32)
    nImgWidth, nImgHeight: the image width and height
*/
NNCAM_API(double)   Nncam_calc_ClarityFactor(const void* pImageData, int bits, unsigned nImgWidth, unsigned nImgHeight);

/*
    nBitCount: output bitmap bit count
    when nBitDepth == 8:
        nBitCount must be 24 or 32
    when nBitDepth > 8
        nBitCount:  24 -> RGB24
                    32 -> RGB32
                    48 -> RGB48
                    64 -> RGB64
*/
NNCAM_API(void)     Nncam_deBayerV2(unsigned nBayer, int nW, int nH, const void* input, void* output, unsigned char nBitDepth, unsigned char nBitCount);

/*
    obsolete, please use Nncam_deBayerV2
*/
NNCAM_DEPRECATED
NNCAM_API(void)     Nncam_deBayer(unsigned nBayer, int nW, int nH, const void* input, void* output, unsigned char nBitDepth);

typedef void (__stdcall* PNNCAM_DEMOSAIC_CALLBACK)(unsigned nBayer, int nW, int nH, const void* input, void* output, unsigned char nBitDepth, void* pCallbackCtx);
NNCAM_API(HRESULT)  Nncam_put_Demosaic(HNncam h, PNNCAM_DEMOSAIC_CALLBACK pCallback, void* pCallbackCtx);

/*
    obsolete, please use NncamModelV2
*/
typedef struct {
#ifdef _WIN32
    const wchar_t*      name;       /* model name, in Windows, we use unicode */
#else
    const char*         name;       /* model name */
#endif
    unsigned            flag;       /* NNCAM_FLAG_xxx */
    unsigned            maxspeed;   /* number of speed level, same as Nncam_get_MaxSpeed(), the speed range = [0, maxspeed], closed interval */
    unsigned            preview;    /* number of preview resolution, same as Nncam_get_ResolutionNumber() */
    unsigned            still;      /* number of still resolution, same as Nncam_get_StillResolutionNumber() */
    NncamResolution   res[NNCAM_MAX];
}NncamModel; /* camera model */

/*
    obsolete, please use NncamDeviceV2
*/
typedef struct {
#ifdef _WIN32
    wchar_t             displayname[64];    /* display name */
    wchar_t             id[64];             /* unique and opaque id of a connected camera, for Nncam_Open */
#else
    char                displayname[64];    /* display name */
    char                id[64];             /* unique and opaque id of a connected camera, for Nncam_Open */
#endif
    const NncamModel* model;
}NncamDevice; /* camera instance for enumerating */

/*
    obsolete, please use Nncam_EnumV2
*/
NNCAM_DEPRECATED
NNCAM_API(unsigned) Nncam_Enum(NncamDevice pti[NNCAM_MAX]);

typedef PNNCAM_DATA_CALLBACK_V3 PNNCAM_DATA_CALLBACK_V2;
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_StartPushModeV2(HNncam h, PNNCAM_DATA_CALLBACK_V2 pDataCallback, void* pCallbackCtx);

typedef void (__stdcall* PNNCAM_DATA_CALLBACK)(const void* pData, const BITMAPINFOHEADER* pHeader, int bSnap, void* pCallbackCtx);
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_StartPushMode(HNncam h, PNNCAM_DATA_CALLBACK pDataCallback, void* pCallbackCtx);

NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_put_ExpoCallback(HNncam h, PINNCAM_EXPOSURE_CALLBACK fnExpoProc, void* pExpoCtx);
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_put_ChromeCallback(HNncam h, PINNCAM_CHROME_CALLBACK fnChromeProc, void* pChromeCtx);

/* Nncam_FfcOnePush is obsolete, it's a synonyms for Nncam_FfcOnce. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_FfcOnePush(HNncam h);

/* Nncam_DfcOnePush is obsolete, it's a synonyms for Nncam_DfcOnce. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_DfcOnePush(HNncam h);

/* Nncam_AwbOnePush is obsolete, it's a synonyms for Nncam_AwbOnce. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_AwbOnePush(HNncam h, PINNCAM_TEMPTINT_CALLBACK fnTTProc, void* pTTCtx);

/* Nncam_AbbOnePush is obsolete, it's a synonyms for Nncam_AbbOnce. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_AbbOnePush(HNncam h, PINNCAM_BLACKBALANCE_CALLBACK fnBBProc, void* pBBCtx);

#ifndef _WIN32

/*
This function is only available on macOS and Linux, it's unnecessary on Windows.
  (1) To process the device plug in / pull out in Windows, please refer to the MSDN
       (a) Device Management, http://msdn.microsoft.com/en-us/library/windows/desktop/aa363224(v=vs.85).aspx
       (b) Detecting Media Insertion or Removal, http://msdn.microsoft.com/en-us/library/windows/desktop/aa363215(v=vs.85).aspx
  (2) To process the device plug in / pull out in Linux / macOS, please call this function to register the callback function.
      When the device is inserted or pulled out, you will be notified by the callback funcion, and then call Nncam_EnumV2(...) again to enum the cameras.
Recommendation: for better rubustness, when notify of device insertion arrives, don't open handle of this device immediately, but open it after delaying a short time (e.g., 200 milliseconds).
*/
typedef void (*PNNCAM_HOTPLUG)(void* pCallbackCtx);
NNCAM_API(void)   Nncam_HotPlug(PNNCAM_HOTPLUG pHotPlugCallback, void* pCallbackCtx);

#else

/* Nncam_Start is obsolete, it's a synonyms for Nncam_StartPushMode. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_Start(HNncam h, PNNCAM_DATA_CALLBACK pDataCallback, void* pCallbackCtx);

/* Nncam_put_TempTintInit is obsolete, it's a synonyms for Nncam_AwbOnce. */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_put_TempTintInit(HNncam h, PINNCAM_TEMPTINT_CALLBACK fnTTProc, void* pTTCtx);

/* ProcessMode: obsolete & useless */
#ifndef __NNCAM_PROCESSMODE_DEFINED__
#define __NNCAM_PROCESSMODE_DEFINED__
#define NNCAM_PROCESSMODE_FULL        0x00    /* better image quality, more cpu usage. this is the default value */
#define NNCAM_PROCESSMODE_FAST        0x01    /* lower image quality, less cpu usage */
#endif
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_put_ProcessMode(HNncam h, unsigned nProcessMode);
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_get_ProcessMode(HNncam h, unsigned* pnProcessMode);

#endif

/* obsolete, please use Nncam_put_Roi and Nncam_get_Roi */
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_put_RoiMode(HNncam h, int bRoiMode, int xOffset, int yOffset);
NNCAM_DEPRECATED
NNCAM_API(HRESULT)  Nncam_get_RoiMode(HNncam h, int* pbRoiMode, int* pxOffset, int* pyOffset);

/* obsolete:
     ------------------------------------------------------------|
     | Parameter         |   Range       |   Default             |
     |-----------------------------------------------------------|
     | VidgetAmount      |   -100~100    |   0                   |
     | VignetMidPoint    |   0~100       |   50                  |
     -------------------------------------------------------------
*/
NNCAM_API(HRESULT)  Nncam_put_VignetEnable(HNncam h, int bEnable);
NNCAM_API(HRESULT)  Nncam_get_VignetEnable(HNncam h, int* bEnable);
NNCAM_API(HRESULT)  Nncam_put_VignetAmountInt(HNncam h, int nAmount);
NNCAM_API(HRESULT)  Nncam_get_VignetAmountInt(HNncam h, int* nAmount);
NNCAM_API(HRESULT)  Nncam_put_VignetMidPointInt(HNncam h, int nMidPoint);
NNCAM_API(HRESULT)  Nncam_get_VignetMidPointInt(HNncam h, int* nMidPoint);

/* obsolete flags */
#define NNCAM_FLAG_BITDEPTH10    NNCAM_FLAG_RAW10  /* pixel format, RAW 10bits */
#define NNCAM_FLAG_BITDEPTH12    NNCAM_FLAG_RAW12  /* pixel format, RAW 12bits */
#define NNCAM_FLAG_BITDEPTH14    NNCAM_FLAG_RAW14  /* pixel format, RAW 14bits */
#define NNCAM_FLAG_BITDEPTH16    NNCAM_FLAG_RAW16  /* pixel format, RAW 16bits */

#ifdef _WIN32
NNCAM_API(HRESULT)  Nncam_put_Name(const wchar_t* id, const char* name);
NNCAM_API(HRESULT)  Nncam_get_Name(const wchar_t* id, char name[64]);
#else
NNCAM_API(HRESULT)  Nncam_put_Name(const char* id, const char* name);
NNCAM_API(HRESULT)  Nncam_get_Name(const char* id, char name[64]);
#endif
NNCAM_API(unsigned) Nncam_EnumWithName(NncamDeviceV2 pti[NNCAM_MAX]);

#ifdef _WIN32
#pragma pack(pop)
#endif

#ifdef __cplusplus
}
#endif

#endif
