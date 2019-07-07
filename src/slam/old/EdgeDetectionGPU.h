
#pragma once

#include "PipelineModule.h"

class EdgeDetectionGPU : public PipelineModule
{
public:

    EdgeDetectionGPU();

    const char* getName() const override;

    size_t getNumPorts() const override;

    bool initialize() override;

    void finalize() override;

    void pushGPU(PipelinePort** ports) override;

    void pullGPU(PipelinePort** ports) override;
};
