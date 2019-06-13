#include "Tracker.h"
#include "TrackerCPUImpl.h"
#include "TrackerGPUImpl.h"

TrackerPtr Tracker::createDefaultTracker()
{
    return std::make_shared<TrackerCPUImpl>();
}

Tracker::Tracker()
{
}

