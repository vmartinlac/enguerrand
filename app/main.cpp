#include <iostream>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Tracker.h"
#include "FileVideoSource.h"
#include "ImageVideoSource.h"

int main(int num_args, char** args)
{
    //cv::Mat input_image = cv::imread("/home/victor/developpement/enguerrand/data/IMG_20190608_120353.jpg");
    //if(input_image.data == nullptr) throw "Could not load image";

    bool go_on = true;

    TrackerPtr tracker = Tracker::createDefaultTracker();

#if 1
    FileVideoSourcePtr video(new FileVideoSource());
    video->setFileName("/home/victor/developpement/enguerrand/data/video3.mp4");
#else
    ImageVideoSourcePtr video(new ImageVideoSource());
    video->load("/home/victor/developpement/enguerrand/data/c.png");
#endif

    go_on = video->open();

    if(go_on)
    {
        video->trigger();
    }

    int count = 0;
    auto t0 = std::chrono::steady_clock::now();

    while(go_on)
    {
        VideoFrame frame;
        video->read(frame);

        go_on = frame.isValid();
        
        if(go_on)
        {
            video->trigger();

            count++;

            cv::Mat image = frame.getView();
            std::cout << "Frame " << frame.getId() << std::endl;

            const double gamma = 0.7;
            cv::resize(image, image, cv::Size(), gamma, gamma);

            std::vector<TrackedLandmark> tracked;
            tracker->track(image, tracked);
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    const float fps = 1.0e3 * float(count) / float(std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count());
    std::cout << "FPS = " << fps << std::endl;

    return 0;
}

