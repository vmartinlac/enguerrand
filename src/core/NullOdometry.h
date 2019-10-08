
#pragma once

#include "OdometryCode.h"

class NullOdometry : public OdometryCode
{
public:

    NullOdometry();
    ~NullOdometry() override;

    bool track(
        double timestamp,
        const std::vector<TrackedCircle>& circles,
        OdometryFrame& output) override;

    void reset() override;
};

