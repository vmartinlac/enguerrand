#pragma once

#include <algorithm>
#include "PipelinePort.h"

class PipelineModule
{
public:

    PipelineModule();

    virtual const char* getName() const = 0;

    virtual size_t getNumPorts() const = 0;

    virtual bool initialize() = 0;

    virtual void finalize() = 0;

    virtual void pushGPU(PipelinePort** ports);

    virtual void pullGPU(PipelinePort** ports);

    virtual void compute(PipelinePort** ports);
};

