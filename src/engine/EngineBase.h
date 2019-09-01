#pragma once

class EngineBase
{
public:

    EngineBase();

    virtual const char* getName() const = 0;

    virtual bool run() = 0;
};

