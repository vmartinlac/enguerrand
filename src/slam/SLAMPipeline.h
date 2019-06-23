#pragma once

#include "VideoSource.h"
#include "PipelineDescription.h"

class SLAMPipeline : public PipelineDescription
{
public:

    SLAMPipeline();

    void build(VideoSourcePtr video);
};

