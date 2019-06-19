#pragma once

#include "SLAMModule.h"

class SLAMModuleWrapper
{
public:

    SLAMModuleWrapper()
    {
        module = nullptr;
        lag = 0;
        next_in_thread = nullptr;
        ports = nullptr;
        enabled = false;
    }

    void executeSequence();

public:

    SLAMModule* module;
    size_t lag;
    SLAMModuleWrapper* next_in_thread;
    SLAMPort** ports;
    bool enabled;
};

