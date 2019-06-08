#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Tracker.h"

int main(int num_args, char** args)
{
    cv::Mat input_image = cv::imread("/home/victor/developpement/enguerrand/data/IMG_20190608_120353.jpg");
    if(input_image.data == nullptr) throw "Could not load image";

    const double gamma = 0.3;
    cv::resize(input_image, input_image, cv::Size(), gamma, gamma);

    TrackerPtr tracker = Tracker::createDefaultTracker();
    
    std::vector<TrackedLandmark> tracked;
    tracker->track(input_image, tracked);

    return 0;
}

