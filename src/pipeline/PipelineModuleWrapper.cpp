#include "PipelineModuleWrapper.h"

void PipelineModuleWrapper::executeSequence()
{
    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        curr->elapsed = PipelineTimeUnit::zero();
    }

    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            std::chrono::time_point< std::chrono::high_resolution_clock > t0 = std::chrono::high_resolution_clock::now();

            curr->module->pushGPU(curr->ports);

            std::chrono::time_point< std::chrono::high_resolution_clock > t1 = std::chrono::high_resolution_clock::now();
            curr->elapsed += std::chrono::duration_cast<PipelineTimeUnit>(t1-t0);
        }
    }

    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            std::chrono::time_point< std::chrono::high_resolution_clock > t0 = std::chrono::high_resolution_clock::now();

            curr->module->compute(curr->ports);

            std::chrono::time_point< std::chrono::high_resolution_clock > t1 = std::chrono::high_resolution_clock::now();
            curr->elapsed += std::chrono::duration_cast<PipelineTimeUnit>(t1-t0);
        }
    }

    for(PipelineModuleWrapper* curr = this; curr!=nullptr; curr=curr->next_in_thread)
    {
        if(curr->enabled)
        {
            std::chrono::time_point< std::chrono::high_resolution_clock > t0 = std::chrono::high_resolution_clock::now();

            curr->module->pullGPU(curr->ports);

            std::chrono::time_point< std::chrono::high_resolution_clock > t1 = std::chrono::high_resolution_clock::now();
            curr->elapsed += std::chrono::duration_cast<PipelineTimeUnit>(t1-t0);
        }
    }
}

