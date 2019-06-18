#pragma once

#include "SLAMModule.h"

class SLAMModuleWrapper
{
public:

    SLAMModuleWrapper()
    {
        lag = 0;
        next_in_thread = nullptr;
        current_frame = nullptr;
    }

    void executeSequence();

public:

    SLAMModulePtr module;
    size_t lag;
    SLAMModuleWrapper* next_in_thread;
    SLAMFrame* current_frame;
};

