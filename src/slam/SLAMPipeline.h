
#pragma once

#include <vector>
#include "SLAMModule.h"

struct SLAMPipelineModule
{
    SLAMModulePtr module;
    size_t lag;
    size_t cpu_thread;
};

class SLAMPipeline
{
public:

    size_t computeLength();

public:

    std::string name;
    size_t num_cpu_threads;
    std::vector<SLAMPipelineModule> modules;
};

typedef std::shared_ptr<SLAMPipeline> SLAMPipelinePtr;
