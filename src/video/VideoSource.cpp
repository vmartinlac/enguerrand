#include "VideoSource.h"

VideoSource::VideoSource()
{
}

void AsynchronousVideoSource::setCallback(const CallbackType& callback)
{
    myCallback = callback;
}

SynchronousVideoSource::SynchronousVideoSource()
{
}

AsynchronousVideoSourcePtr AsynchronousVideoSource::asAsynchronous()
{
    return AsynchronousVideoSourcePtr(this);
}

SynchronousVideoSourcePtr AsynchronousVideoSource::asSynchronous()
{
    return SynchronousVideoSourcePtr();
}

VideoSource::SynchronicityType AsynchronousVideoSource::getSynchronicity()
{
    return ASYNCHRONOUS;
}

AsynchronousVideoSource::AsynchronousVideoSource()
{
}

AsynchronousVideoSourcePtr SynchronousVideoSource::asAsynchronous()
{
    return AsynchronousVideoSourcePtr();
}

SynchronousVideoSourcePtr SynchronousVideoSource::asSynchronous()
{
    return SynchronousVideoSourcePtr(this);
}

VideoSource::SynchronicityType SynchronousVideoSource::getSynchronicity()
{
    return SYNCHRONOUS;
}

