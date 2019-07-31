#pragma once

#include <tbb/flow_graph.h>
#include <thread>
#include <mutex>
#include "EngineConfig.h"

class Engine
{
public:

    bool exec(EngineConfigPtr config);
};
