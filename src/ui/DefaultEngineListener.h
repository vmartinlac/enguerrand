#pragma once

#include "EngineListener.h"

class DefaultEngineListener : public EngineListener
{
public:

    DefaultEngineListener();

    void operator()(size_t frame_id, double timestamp) override;
};

