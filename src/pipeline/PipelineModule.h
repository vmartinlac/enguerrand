#pragma once

#include <algorithm>
#include "PipelinePort.h"

class PipelineModule
{
public:

    PipelineModule();

    virtual size_t getNumPorts() const = 0;

    virtual bool initialize() = 0;

    virtual void finalize() = 0;

    virtual void pushGPU(PipelinePort** ports);

    virtual void pullGPU(PipelinePort** ports);

    virtual void compute(PipelinePort** ports);
};

