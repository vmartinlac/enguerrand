#include <iostream>
#include "FileVideoSource.h"
#include "Engine.h"

int main(int num_args, char** args)
{
    Engine engine;
    FileVideoSourcePtr video_source(new FileVideoSource());

    /*
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190608_120103.mp4");
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190608_120231.mp4");
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190615_150222.mp4");
    */
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190615_180529.mp4");

    if( video_source->open() == false ) exit(1);

    engine.start();

    bool go_on = true;
    while(go_on)
    {
        VideoFrame f;

        video_source->trigger();
        video_source->read(f);

        if(f.isValid())
        {
            engine.grabFrame(std::move(f), false);
        }
        else
        {
            go_on = false;
        }
    }

    engine.stop();

    video_source->close();

    return 0;
}

