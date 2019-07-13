#include <iostream>
#include "FileVideoSource.h"
#include "EKFOdometry.h"
#include "Engine.h"

int main(int num_args, char** args)
{
    EngineConfigPtr config(new EngineConfig());

    {
        FileVideoSourcePtr video_source(new FileVideoSource());

        const std::string data_dir = "/data/victor/enguerrand_data/sample_videos/";

        const std::string video_filename = "VID_20190615_180529.mp4";
        //const std::string video_filename = "VID_20190608_120103.mp4";
        //const std::string video_filename = "VID_20190608_120231.mp4";
        //const std::string video_filename = "VID_20190615_150222.mp4";

        video_source->setFileName(data_dir + "/" + video_filename);

        config->video = video_source;
    }

    config->odometry_code.reset(new EKFOdometry());

    {
        const std::string path = "/data/victor/enguerrand_data/balls_histogram.bin";

        config->balls_reference_histogram.reset(new Histogram());

        if( config->balls_reference_histogram->load(path) == false )
        {
            exit(1);
        }
    }

    Engine engine;
    engine.exec(config);

    return 0;
}

