#include "SLAMWorkLoad.h"

void SLAMWorkLoad::execute()
{
    for(SLAMWorkItem& item : work_items)
    {
        if(item.frame && item.frame->header.ready)
        {
            item.module->pushGPU(item.frame);
        }
    }

    for(SLAMWorkItem& item : work_items)
    {
        if(item.frame && item.frame->header.ready)
        {
            item.module->computeCPU(item.frame);
        }
    }

    for(SLAMWorkItem& item : work_items)
    {
        if(item.frame && item.frame->header.ready)
        {
            item.module->pullGPU(item.frame);
        }
    }
}

