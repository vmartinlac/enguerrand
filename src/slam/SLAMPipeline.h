
#pragma once

#include <memory>
#include <vector>
#include "SLAMModule.h"

class SLAMPipeline
{
public:

    std::vector< std::unique_ptr<SLAMModule> > modules;
    std::vector< std::function<SLAMPort*()> > meta_ports;
    std::vector<size_t> connections;

    std::vector<size_t> lags;
    std::vector<size_t> thread_partition;
};

