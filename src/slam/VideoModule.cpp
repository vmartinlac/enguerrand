#include <iostream>
#include "VideoPort.h"
#include "VideoModule.h"

VideoModule::VideoModule()
{
}

const char* VideoModule::getName() const
{
    return "VideoModule";
}

void VideoModule::setVideoSource(VideoSourcePtr video)
{
    mVideo = std::move(video);
}

size_t VideoModule::getNumPorts() const
{
    return 1;
}

bool VideoModule::initialize()
{
    mTriggered = false;

    if(mVideo)
    {
        return mVideo->open();
    }
    else
    {
        return false;
    }
}

void VideoModule::finalize()
{
    mVideo->close();
}

void VideoModule::compute(PipelinePort** ports)
{
    VideoPort* port = static_cast<VideoPort*>(ports[0]);

    if(mTriggered == false)
    {
        mVideo->trigger();
        mTriggered = true;
    }
    
    mVideo->read(port->frame);

    /*
    if(port->frame.isValid())
    std::cout << "Reading frame " << port->frame.getId() << std::endl;
    */

    mVideo->trigger();
}

