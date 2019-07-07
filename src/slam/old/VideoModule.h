#pragma once

#include "VideoSource.h"
#include "PipelineModule.h"

class VideoModule : public PipelineModule
{
public:

    VideoModule();

    const char* getName() const;

    void setVideoSource(VideoSourcePtr video);

    size_t getNumPorts() const override;

    bool initialize() override;

    void finalize() override;

    void compute(PipelinePort** ports) override;

protected:

    bool mTriggered;
    VideoSourcePtr mVideo;
};

