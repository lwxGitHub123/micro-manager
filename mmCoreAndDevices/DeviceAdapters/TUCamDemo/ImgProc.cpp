#include "ImgProc.h"
#include <opencv2\opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include<windows.h>




using namespace std;
using namespace cv;




// wchar_t to string
void CImgProc::Wchar_tToString(std::string& szDst, wchar_t *wchar)
{
    wchar_t * wText = wchar;
    DWORD dwNum = WideCharToMultiByte(CP_OEMCP,NULL,wText,-1,NULL,0,NULL,FALSE);// WideCharToMultiByte的运用
    char *psText; // psText为char*的临时数组，作为赋值给std::string的中间变量
    psText = new char[dwNum];
    WideCharToMultiByte (CP_OEMCP,NULL,wText,-1,psText,dwNum,NULL,FALSE);// WideCharToMultiByte的再次运用
    szDst = psText;// std::string赋值
    delete []psText;// psText的清除
}


wchar_t* CImgProc::CharToWchar(const char* c)  
{  
     
    int len = MultiByteToWideChar(CP_ACP,0,c,strlen(c),NULL,0);  
    wchar_t* m_wchar=new wchar_t[len+1];  
    MultiByteToWideChar(CP_ACP,0,c,strlen(c),m_wchar,len);  
    m_wchar[len]='\0';  
    return m_wchar;  
} 


wchar_t* CImgProc::StringToWchar(const string& s)  
{  
    const char* p=s.c_str();  
    return CharToWchar(p);  
} 


Mat CImgProc::Rgb24ToMat(void* g_pImageData,unsigned int height,unsigned int width)
{
	cv::Mat dst_mat(height, width, CV_8UC3);
    memcpy(dst_mat.data,g_pImageData , height*width*3*sizeof(unsigned char));

	return dst_mat;
}



//************************************
// Method:    TransBufferToMat
// FullName:  图片buffer数据转换成 Mat数据格式;
// Access:    public 
// Returns:   cv::Mat
// Qualifier:
// Parameter: unsigned char * pBuffer  图片数据内容
// Parameter: int nWidth	图片的宽度
// Parameter: int nHeight  图片的高度
// Parameter: int nBandNum 每个像素包含数据个数 (1, 3, 4 ) 1:Gray 3:RGB 4:RGBA
// Parameter: int nBPB  每个像素数据 所占位数(1, 2) 1:8位  2:16位;
//************************************
Mat CImgProc::TransBufferToMat(unsigned char* pBuffer, int nWidth, int nHeight, int nBandNum, int nBPB )
{
	cv::Mat mDst;
	if (nBandNum == 4)
	{
		if (nBPB == 1)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_8UC4);
		}
		else if (nBPB == 2)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_16UC4);
		}
	}
	else if (nBandNum == 3)
	{
		if (nBPB == 1)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_8UC3);
		}
		else if (nBPB == 2)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_16UC3);
		}
	}
	else if (nBandNum == 1)
	{
		if (nBPB == 1)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_8UC1);
		}
		else if (nBPB == 2)
		{
			mDst = cv::Mat::zeros(cv::Size(nWidth, nHeight), CV_16UC1);
		}
	}

	for (int j = 0; j < nHeight; ++j)
	{
		unsigned char* data = mDst.ptr<unsigned char>(j);
		unsigned char* pSubBuffer = pBuffer + (nHeight - 1 - j) * nWidth* nBandNum*nBPB;
		memcpy(data, pSubBuffer, nWidth*nBandNum*nBPB);
	}
	if (nBandNum == 1)
	{
		cv::cvtColor(mDst, mDst, COLOR_GRAY2BGR);
	}
	else if (nBandNum == 3)
	{
		cv::cvtColor(mDst, mDst, COLOR_RGB2BGR);
	}
	else if (nBandNum == 4)
	{
		cv::cvtColor(mDst, mDst, COLOR_RGBA2BGR);
	}

	return mDst;
}



