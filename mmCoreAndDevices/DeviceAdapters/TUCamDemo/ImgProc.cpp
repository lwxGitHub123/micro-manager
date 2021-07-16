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
    DWORD dwNum = WideCharToMultiByte(CP_OEMCP,NULL,wText,-1,NULL,0,NULL,FALSE);// WideCharToMultiByte������
    char *psText; // psTextΪchar*����ʱ���飬��Ϊ��ֵ��std::string���м����
    psText = new char[dwNum];
    WideCharToMultiByte (CP_OEMCP,NULL,wText,-1,psText,dwNum,NULL,FALSE);// WideCharToMultiByte���ٴ�����
    szDst = psText;// std::string��ֵ
    delete []psText;// psText�����
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




