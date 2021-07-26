#pragma once
#include "Utils.h"
#include<iostream>
#include <stdio.h>
#include <string>
#include <time.h>
#include <io.h>
#include <direct.h>
#include <stdlib.h>
#include <windows.h>



#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif


using namespace std;


string CUtils::getCwd(){  
    //获取当前工作目录  
    string path;  
    path = getcwd(NULL,0);  
    return path;  
}  

	//在文件夹下创建cameraLog文件夹
string CUtils::createDirectory(string directoryPath)
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

void CUtils::cameraLog(string log)
{
	string path = getCwd();  
    cout << path << endl;  
	string logFile = createDirectory(path);
	ofstream File;
	SYSTEMTIME st = { 0 };  
	GetLocalTime(&st);  //获取当前时间 可精确到ms
	string nowStr = to_string(static_cast<long long>(st.wYear)) +"-"+ to_string(static_cast<long long>(st.wMonth)) +"-"+ to_string(static_cast<long long>(st.wDay)) +"  "+ to_string(static_cast<long long>(st.wHour)) + ":"+ to_string(static_cast<long long>(st.wMinute))
		+ ":" +to_string(static_cast<long long>(st.wSecond)) + "." +to_string(static_cast<long long>(st.wMilliseconds));
	File.open(logFile + "\\log"+".txt",ios::out | ios::app);
	File<<nowStr + "     " + log<<endl;
	File.close();

}


