#include "Utils.h"

#include <stdio.h>
#include <direct.h>
#include <io.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <cstdio> 
#include <string>
#include <math.h>
#include "Utils.h"
#include <windows.h>

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif


using namespace std;


string getCwd(){  
    //��ȡ��ǰ����Ŀ¼  
    string path;  
    path = getcwd(NULL,0);  
    return path;  
}  

//���ļ����´���cameraLog�ļ���
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
	GetLocalTime(&st);  //��ȡ��ǰʱ�� �ɾ�ȷ��ms
	string nowStr = to_string(static_cast<long long>(st.wYear)) +"-"+ to_string(static_cast<long long>(st.wMonth)) +"-"+ to_string(static_cast<long long>(st.wDay)) +"  "+ to_string(static_cast<long long>(st.wHour)) + ":"+ to_string(static_cast<long long>(st.wMinute))
		+ ":" +to_string(static_cast<long long>(st.wSecond)) + "." +to_string(static_cast<long long>(st.wMilliseconds));
	File.open(logFile + "\\log"+".txt",ios::out | ios::app);
	File<<nowStr + "     " + log<<endl;
	File.close();

	//return logFile ;
}

Utils::Utils(void)
{
}


Utils::~Utils(void)
{
}
