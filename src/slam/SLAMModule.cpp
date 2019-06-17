#include "SLAMModule.h"

SLAMModule::SLAMModule(SLAMModuleType type)
{
    mType = type;
}

void SLAMModule::pushGPU(SLAMFramePtr frame)
{
}

void SLAMModule::pullGPU(SLAMFramePtr frame)
{
}

void SLAMModule::computeCPU(SLAMFramePtr frame)
{
}

