#pragma once

#include <stdio.h>
#include <direct.h>
#include <io.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;


#if !defined _UTILS_H_ //��ֹNum_add.h�ڱ���ļ���������
#define _UTILS_H_ 

class CUtils
{
private :

	//static unsigned g_total = 0 ;

public:

	static string getCwd();

	static string createDirectory(string directoryPath);

	static void cameraLog(string log);

};

#endif  //_UTILS_H_

