#include "FileVideoSource.h"
#include "ImageVideoSource.h"
#include "PipelineEngine.h"
#include "SLAMPipeline.h"

int main(int num_args, char** args)
{
    FileVideoSourcePtr video(new FileVideoSource());
    video->setFileName("/home/victor/developpement/enguerrand/data/video3.mp4");

    //ImageVideoSourcePtr video(new ImageVideoSource());
    //video->load("/home/victor/developpement/enguerrand/data/c.png");

    SLAMPipeline pipeline;
    pipeline.build(video);

    PipelineEngine engine;
    engine.exec(pipeline);

    return 0;
}

