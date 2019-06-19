#pragma once

#include <algorithm>
#include "SLAMPort.h"

class SLAMModule
{
public:

    SLAMModule();

    virtual size_t getNumPorts() const = 0;

    virtual size_t getPortType(size_t input) const = 0;

    virtual void initialize();

    virtual void finalize();

    virtual void pushGPU(SLAMPort** ports);

    virtual void pullGPU(SLAMPort** ports);

    virtual void compute(SLAMPort** ports);
};

