
#pragma once

#include <functional>
#include "VideoFrame.h"

class SynchronousVideoSource;
class AsynchronousVideoSource;

using SynchronousVideoSourcePtr = std::shared_ptr<SynchronousVideoSource>;
using AsynchronousVideoSourcePtr = std::shared_ptr<AsynchronousVideoSource>;

class VideoSource : public std::enable_shared_from_this<VideoSource>
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

    virtual AsynchronousVideoSourcePtr asAsynchronous() = 0;

    virtual SynchronousVideoSourcePtr asSynchronous() = 0;

    virtual int getNumViews() = 0;
};

using VideoSourcePtr = std::shared_ptr<VideoSource>;

class AsynchronousVideoCallback
{
public:

    AsynchronousVideoCallback();

    virtual void operator()(VideoFrame&&) = 0;
};

class AsynchronousVideoSource : public VideoSource
{
public:

    AsynchronousVideoSource();

    AsynchronousVideoSourcePtr asAsynchronous() final;

    SynchronousVideoSourcePtr asSynchronous() final;

    SynchronicityType getSynchronicity() final;

    void setCallback(AsynchronousVideoCallback* callback);

    virtual bool start() = 0;

    virtual void stop() = 0;

protected:

    AsynchronousVideoCallback* myCallback;
};

class SynchronousVideoSource : public VideoSource
{
public:

    SynchronousVideoSource();

    AsynchronousVideoSourcePtr asAsynchronous() final;

    SynchronousVideoSourcePtr asSynchronous() final;

    SynchronicityType getSynchronicity() final;

    virtual bool open() = 0;

    virtual void close() = 0;

    virtual void read(VideoFrame& frame) = 0;

    virtual int getNumViews() = 0;
};

