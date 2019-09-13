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
    return std::static_pointer_cast<AsynchronousVideoSource>(shared_from_this());
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
    return std::static_pointer_cast<SynchronousVideoSource>(shared_from_this());
}

VideoSource::SynchronicityType SynchronousVideoSource::getSynchronicity()
{
    return SYNCHRONOUS;
}

