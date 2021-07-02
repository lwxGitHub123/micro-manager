#include "PpParam.h"

PpParam::PpParam(std::string name, int ppIndex, int propIndex) : mIsBoolean(false)
{
    mName = name, mppIndex = ppIndex, mpropIndex = propIndex, mcurValue = ppIndex;
}

std::string PpParam::GetName()        { return mName; }
int         PpParam::GetppIndex()     { return mppIndex; }
int         PpParam::GetpropIndex()   { return mpropIndex; }
bool        PpParam::IsBoolean()      { return mIsBoolean; }
double      PpParam::GetcurValue()    { return mcurValue; }
void        PpParam::SetName(std::string name)    { mName      = name; }
void        PpParam::SetppIndex(int ppIndex)      { mppIndex   = ppIndex; }
void        PpParam::SetpropInex(int propIndex)   { mpropIndex = propIndex; }
void        PpParam::SetcurValue(double curValue) { mcurValue  = curValue; }
void        PpParam::SetBoolean(bool isBoolean)   { mIsBoolean = isBoolean; }

void PpParam::SetPostProc(PpParam& tmp)
{
    mName = tmp.GetName(), mppIndex = tmp.GetppIndex(), mpropIndex = tmp.GetpropIndex();
}