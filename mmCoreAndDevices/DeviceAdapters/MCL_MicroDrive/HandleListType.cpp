/*
File:		HandleListType.cpp
Copyright:	Mad City Labs Inc., 2019
License:	Distributed under the BSD license.
*/
#include "HandleListType.h"
#include <stdio.h>
#include <string>

HandleListType::HandleListType(int handle, int type, int axis1, int axis2)
{
	handle_ = handle;
	type_ = type;
	axis1_ = axis1;
	axis2_ = axis2;
}

HandleListType::~HandleListType()
{
}

int HandleListType::getType()
{
	return type_;
}

int HandleListType::getHandle()
{
	return handle_;
}

void HandleListType::setType(int type)
{
	type_ = type;
}

void HandleListType::setHandle(int handle)
{
	handle_ = handle;
}

void HandleListType::setAxis1(int axis)
{
	axis1_ = axis;
}

void HandleListType::setAxis2(int axis)
{
	axis2_ = axis;
}

int HandleListType::getAxis1()
{
	return axis1_;
}

int HandleListType::getAxis2()
{
	return axis2_;
}

bool HandleListType::IsControlling(int handle, int a1, int a2)
{
	bool isControlling = false;
	if (handle == handle_)
	{
		int conflicts = 0;
		if (a1 != 0)
		{
			if (a1 == axis1_ || a1 == axis2_)
				conflicts++;
		}
		if (a2 != 0)
		{
			if (a2 == axis1_ || a2 == axis2_)
				conflicts++;
		}
		isControlling = conflicts != 0;
	}
	return isControlling;
}

bool HandleListType::operator==(const HandleListType &rhs)
{
	if (handle_ == rhs.handle_ && type_ == rhs.type_ &&
		axis1_ == rhs.axis1_ && axis2_ == rhs.axis2_)
		return true;

	return false;
}