
#pragma once

#include <functional>
#include <memory>
#include <vector>
#include "PipelineModule.h"
#include "PipelinePort.h"

class PipelineDescription
{
public:

    std::vector< std::unique_ptr<PipelineModule> > modules;
    std::vector< std::unique_ptr<PipelinePortFactory> > ports;
    std::vector<size_t> connections;
    std::vector<size_t> lags;
    std::vector<size_t> thread_partition;
};

