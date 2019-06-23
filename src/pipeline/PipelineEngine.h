
#pragma once

#include <sophus/se3.hpp>
#include "PipelineModule.h"
#include "PipelineThread.h"
#include "PipelineDescription.h"

class PipelineEngine
{
public:

    PipelineEngine();

    void exec(PipelineDescription& pipeline);

protected:

    std::vector<PipelineModuleWrapper> mModules;
    std::vector< std::unique_ptr<PipelineThread> > mThreads;
    std::vector< std::unique_ptr<PipelinePort> > mPorts;
    std::vector<PipelinePort*> mPortTable;
    size_t mPipelineLength;
    size_t mCycleOffset;
};

