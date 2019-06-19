#pragma once

#include <sophus/se3.hpp>
#include "SLAMModule.h"
#include "SLAMThread.h"
#include "SLAMPipeline.h"

class SLAMEngine
{
public:

    SLAMEngine();

    void exec(SLAMPipeline& pipeline);

protected:

    std::vector<SLAMModuleWrapper> mModules;
    std::vector< std::unique_ptr<SLAMThread> > mThreads;
    std::vector< std::unique_ptr<SLAMPort> > mPorts;
    std::vector<SLAMPort*> mPortTable;
    size_t mPipelineLength;
    size_t mCycleOffset;
};

