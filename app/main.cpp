#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Tracker.h"
#include "FileVideoSource.h"

int main(int num_args, char** args)
{
    //cv::Mat input_image = cv::imread("/home/victor/developpement/enguerrand/data/IMG_20190608_120353.jpg");
    //if(input_image.data == nullptr) throw "Could not load image";

    bool go_on = true;

    TrackerPtr tracker = Tracker::createDefaultTracker();

    FileVideoSourcePtr video(new FileVideoSource());

    video->setFileName("/home/victor/developpement/enguerrand/data/VID_20190608_120231.mp4");

    go_on = video->open();

    if(go_on)
    {
        video->trigger();
    }

    while(go_on)
    {
        VideoFrame frame;
        video->read(frame);

        go_on = frame.isValid();
        
        if(go_on)
        {
            video->trigger();

            cv::Mat image = frame.getView();
            std::cout << "Frame " << frame.getId() << std::endl;

            const double gamma = 0.7;
            cv::resize(image, image, cv::Size(), gamma, gamma);

            std::vector<TrackedLandmark> tracked;
            tracker->track(image, tracked);
        }
    }

    return 0;
}

