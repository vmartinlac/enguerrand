#include "SLAMModuleWrapper.h"

void SLAMModuleWrapper::executeSequence()
{
    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->current_frame && curr->current_frame->header.ready)
        {
            curr->module->pushGPU(curr->current_frame);
        }
    }

    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->current_frame && curr->current_frame->header.ready)
        {
            curr->module->computeCPU(curr->current_frame);
        }
    }

    for(SLAMModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->current_frame && curr->current_frame->header.ready)
        {
            curr->module->pullGPU(curr->current_frame);
        }
    }
}

