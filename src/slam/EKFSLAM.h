
#pragma once

#include "PipelineModule.h"

class EKFSLAM : public PipelineModule
{
public:

    EKFSLAM();

    size_t getNumPorts() const override;

    bool initialize() override;

    void finalize() override;

    void compute(PipelinePort** ports) override;
};

