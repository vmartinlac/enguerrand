
#pragma once

#include <vector>
#include "SLAMModule.h"

class SLAMPipeline
{
public:

    std::vector<SLAMModulePtr> modules;
    std::vector<size_t> lags;
    std::vector<size_t> thread_partition;

    // virtual SLAMFramePtr createFrame() = 0;
};

