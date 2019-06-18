#pragma once

#include <memory>
#include "SLAMFrame.h"

class SLAMModule
{
public:

        SLAMModule();

        //virtual void init() = 0; // TODO

        virtual void pushGPU(SLAMFrame* frame);
        virtual void pullGPU(SLAMFrame* frame);

        virtual void computeCPU(SLAMFrame* frame);
};

typedef std::shared_ptr<SLAMModule> SLAMModulePtr;

