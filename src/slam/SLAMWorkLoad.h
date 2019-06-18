#pragma once

#include "SLAMModule.h"

struct SLAMWorkItem
{
    SLAMModulePtr module;
    SLAMFrame* frame;
};

struct SLAMWorkLoad
{
    std::vector<SLAMWorkItem> work_items;

    void execute();
};

