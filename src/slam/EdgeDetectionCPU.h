
#pragma once

#include "PipelineModule.h"

class EdgeDetectionCPU : public PipelineModule
{
public:

    EdgeDetectionCPU();

    const char* getName() const override;

    size_t getNumPorts() const override;

    bool initialize() override;

    void finalize() override;

    void compute(PipelinePort** ports) override;

protected:

    std::array<cv::Vec2i,8> mNeighbors;
};
