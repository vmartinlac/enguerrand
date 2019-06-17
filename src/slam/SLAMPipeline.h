
#pragma once

#include <vector>
#include "SLAMModule.h"

struct SLAMPipelineModule
{
    SLAMModuleSPtr module;
    size_t lag;
};

class SLAMPipeline
{
public:

    SLAMPipeline();

    std::string getName();

    size_t getNumModules();

    size_t getNumGpuModules();

    SLAMModulePtr getGpuModules(size_t i);

    size_t getGpuModuleLag(size_t i);

    size_t computeLength();

protected:

    std::string name;
    std::vector<SLAMPipelineModule> gpu_modules;
    std::vector< std::vector<SLAMPipelineModule> > cpu_modules;
};

typedef std::shared_ptr<SLAMPipeline> SLAMPipelineSPtr;
