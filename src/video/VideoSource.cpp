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

AsynchronousVideoSource* AsynchronousVideoSource::asAsynchronous()
{
    return this;
}

SynchronousVideoSource* AsynchronousVideoSource::asSynchronous()
{
    return nullptr;
}

VideoSource::SynchronicityType AsynchronousVideoSource::getSynchronicity()
{
    return ASYNCHRONOUS;
}

AsynchronousVideoSource::AsynchronousVideoSource()
{
}

AsynchronousVideoSource* SynchronousVideoSource::asAsynchronous()
{
    return nullptr;
}

SynchronousVideoSource* SynchronousVideoSource::asSynchronous()
{
    return this;
}

VideoSource::SynchronicityType SynchronousVideoSource::getSynchronicity()
{
    return SYNCHRONOUS;
}

