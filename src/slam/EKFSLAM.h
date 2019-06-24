
#pragma once

#include "PipelineModule.h"

class EKFSLAM : public PipelineModule
{
public:

    EKFSLAM();

    const char* getName() const override;

    size_t getNumPorts() const override;

    bool initialize() override;

    void finalize() override;

    void compute(PipelinePort** ports) override;
};

