
#pragma once

#include <Eigen/Eigen>
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

protected:

    void predict();

    void update();

protected:

    bool mFirst;
    size_t mMaxLocalMapSize;
    size_t mLocalMapSize;
    Eigen::VectorXd mMu;
    Eigen::MatrixXd mSigma;
};

