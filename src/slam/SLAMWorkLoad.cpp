#include "SLAMWorkLoad.h"

void SLAMWorkLoad::execute()
{
    for(SLAMWorkItem& item : work_items)
    {
        item.module->pushGPU(item.frame);
    }

    for(SLAMWorkItem& item : work_items)
    {
        item.module->computeCPU(item.frame);
    }

    for(SLAMWorkItem& item : work_items)
    {
        item.module->pullGPU(item.frame);
    }
}

