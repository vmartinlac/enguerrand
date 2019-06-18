#include "SLAMPipeline.h"

size_t SLAMPipeline::computeLength()
{
    size_t max_lag = 0;

    for( SLAMPipelineModule& mod : pipeline->modules )
    {
        max_lag = std::max(max_lag, mod.lag);
    }

    return max_lag + 1;
}

