
#pragma once

#include <chrono>
#include <QObject>
#include "EngineListener.h"

class DefaultEngineListener : public EngineListener
{
public:

    DefaultEngineListener(QObject* receiver);

    void operator()(size_t frame_id, double timestamp) override;

protected:

    using ClockType = std::chrono::high_resolution_clock;

protected:

    bool myFirst;
    std::chrono::milliseconds myInhibitionTime;
    ClockType::time_point myLastFrame;
    QObject* myReceiver;
};

