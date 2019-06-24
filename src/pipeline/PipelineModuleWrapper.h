#pragma once

#include "PipelineUtils.h"
#include "PipelineModule.h"

class PipelineModuleWrapper
{
public:

    PipelineModuleWrapper()
    {
        module = nullptr;
        lag = 0;
        next_in_thread = nullptr;
        ports = nullptr;
        enabled = false;
    }

    void executeSequence();

public:

    PipelineModule* module;
    size_t lag;
    PipelineModuleWrapper* next_in_thread;
    PipelinePort** ports;
    bool enabled;
    PipelineTimeUnit elapsed;
};

