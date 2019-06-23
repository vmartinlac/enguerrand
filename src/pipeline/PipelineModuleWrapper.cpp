#include "PipelineModuleWrapper.h"

void PipelineModuleWrapper::executeSequence()
{
    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->pushGPU(curr->ports);
        }
    }

    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->compute(curr->ports);
        }
    }

    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            curr->module->pullGPU(curr->ports);
        }
    }
}

