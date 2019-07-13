#include <iostream>
#include "FileVideoSource.h"
#include "EKFOdometry.h"
#include "Engine.h"

int main(int num_args, char** args)
{
    FileVideoSourcePtr video_source(new FileVideoSource());

    const std::string data_dir = "/data/victor/enguerrand_data/";

    const std::string video_filename = "VID_20190615_180529.mp4";
    //const std::string video_filename = "VID_20190608_120103.mp4";
    //const std::string video_filename = "VID_20190608_120231.mp4";
    //const std::string video_filename = "VID_20190615_150222.mp4";

    video_source->setFileName(data_dir + "/" + video_filename);

    OdometryCodePtr odometry_code(new EKFOdometry());

    Engine engine;

    engine.exec(video_source, odometry_code);

    return 0;
}

