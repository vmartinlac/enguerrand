#include "Tracker.h"
#include "TrackerImpl.h"

TrackerPtr Tracker::createDefaultTracker()
{
    return std::make_shared<TrackerImpl>();
}

Tracker::Tracker()
{
}

