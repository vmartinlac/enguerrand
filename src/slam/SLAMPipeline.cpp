#include "SLAMPipeline.h"

size_t SLAMPipeline::computeLength()
{
    size_t max_lag = 0;

    for( SLAMPipelineModule& mod : pipeline->gpu_modules )
    {
        max_lag = std::max(max_lag, mod.lag);
    }

    for( std::vector<SLAMPipelineModule>& mods : pipeline->cpu_modules)
    {
        for( SLAMPipelineModule& mod : mods )
        {
            max_lag = std::max(max_lag, mod.lag);
        }
    }

    return max_lag + 1;
}

