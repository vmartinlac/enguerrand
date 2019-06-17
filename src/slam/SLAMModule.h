#pragma once

#include "SLAMFrame.h"

enum SLAMModuleType
{
    SLAM_MODULE_CPU,
    SLAM_MODULE_GPU
};

class SLAMModule
{
public:

        SLAMModule(SLAMModuleType type);

        //virtual void init() = 0; // TODO

        SLAMModuleType getType();

        virtual void pushGPU(SLAMFramePtr frame);
        virtual void pullGPU(SLAMFramePtr frame);

        virtual void computeCPU(SLAMFramePtr frame);

private:

    SLAMModuleType mType;
};

typedef std::shared_ptr<SLAMModule> SLAMModulePtr;

inline SLAMModuleType SLAMModule::getType()
{
    return mType;
}

