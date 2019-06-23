#pragma once

class PipelinePort
{
public:

    PipelinePort();

    virtual void reset() = 0;
};

class PipelinePortFactory
{
public:

    PipelinePortFactory()
    {
    }

    virtual PipelinePort* create() = 0;
};

template<typename PortType>
class GenericPipelinePortFactory : public PipelinePortFactory
{
public:

    GenericPipelinePortFactory()
    {
    }

    PipelinePort* create() override
    {
        return new PortType();
    }
};


