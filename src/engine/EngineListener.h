
#pragma once

#include <cstddef>

class EngineListener
{
public:

    EngineListener();

    virtual void operator()(size_t frame_id, double timestamp) = 0; 
};

