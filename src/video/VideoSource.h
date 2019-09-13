
#pragma once

#include <functional>
#include "VideoFrame.h"

class SynchronousVideoSource;
class AsynchronousVideoSource;

class VideoSource
{
public:

    enum SynchronicityType
    {
        SYNCHRONOUS,
        ASYNCHRONOUS,
    };

public:

    VideoSource();

    virtual SynchronicityType getSynchronicity() = 0;

    virtual AsynchronousVideoSource* asAsynchronous() = 0;

    virtual SynchronousVideoSource* asSynchronous() = 0;

    virtual int getNumViews() = 0;
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;

class AsynchronousVideoSource : public VideoSource
{
public:

    using CallbackType = std::function<void(VideoFrame&&)>;

public:

    AsynchronousVideoSource();

    AsynchronousVideoSource* asAsynchronous() final;

    SynchronousVideoSource* asSynchronous() final;

    SynchronicityType getSynchronicity() final;

    void setCallback(const CallbackType&);

    virtual bool start() = 0;

    virtual void stop() = 0;

protected:

    CallbackType myCallback;
};

class SynchronousVideoSource : public VideoSource
{
public:

    SynchronousVideoSource();

    AsynchronousVideoSource* asAsynchronous() final;

    SynchronousVideoSource* asSynchronous() final;

    SynchronicityType getSynchronicity() final;

    virtual bool open() = 0;

    virtual void close() = 0;

    virtual void trigger() = 0;

    virtual void read(VideoFrame& frame) = 0;

    virtual int getNumViews() = 0;
};

