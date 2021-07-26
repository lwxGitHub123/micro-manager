#pragma once
#include <direct.h>
#include <io.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <string>


#include<windows.h>
#include <opencv2\opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp> 


using namespace std;

using namespace cv;


#if !defined _IMGPROC_H_ //防止ImgProc.h在别的文件里多次声明
#define _IMGPROC_H_ 


class CImgProc
{

private :



public:

	static void Wchar_tToString(std::string& szDst, wchar_t *wchar);

	static wchar_t* CharToWchar(const char* c) ;

	static wchar_t* StringToWchar(const string& s) ;

	

	static Mat Rgb24ToMat(void* g_pImageData,unsigned int height,unsigned int width);


	static Mat TransBufferToMat(unsigned char* pBuffer, int nWidth, int nHeight, int nBandNum, int nBPB);


};

#endif  //_IMGPROC_H_
