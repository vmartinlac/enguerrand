#include "SLAMModuleWrapper.h"

void SLAMModuleWrapper::executeSequence()
{
    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->pushGPU(curr->ports);
        }
    }

    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->compute(curr->ports);
        }
    }

    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->pullGPU(curr->ports);
        }
    }
}

