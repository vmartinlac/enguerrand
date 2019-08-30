#include "DefaultEngineListener.h"

DefaultEngineListener::DefaultEngineListener(QObject* receiver)
{
    myFirst = true;
    myReceiver = receiver;
    myInhibitionTime = std::chrono::milliseconds(15);
}

void DefaultEngineListener::operator()(size_t frame_id, double timestamp)
{
    ClockType::time_point now = ClockType::now();

    if(myFirst || myLastFrame + myInhibitionTime < now)
    {
        myFirst = false;
        myLastFrame = now;

        QMetaObject::invokeMethod(myReceiver, "handleFrame", Qt::QueuedConnection); // use blocking call?
    }
}

