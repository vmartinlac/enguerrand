#include <iostream>
#include "RealsenseVideoSource.h"

RealsenseVideoSource::RealsenseVideoSource(rs2::sensor sensor, rs2::stream_profile profile)
{
    mySensor = sensor;
    myProfile = profile;
}

bool RealsenseVideoSource::start()
{
    auto proc = [this] (rs2::frame frame)
    {
        if(frame.is<rs2::video_frame>() && frame.get_profile().format() == RS2_FORMAT_BGR8)
        {
            const ClockType::time_point now = ClockType::now();

            rs2::video_frame vframe = frame.as<rs2::video_frame>();

            cv::Mat image = cv::Mat(
                vframe.get_height(),
                vframe.get_width(),
                CV_8UC3,
                const_cast<void*>( vframe.get_data() ),
                vframe.get_stride_in_bytes()).clone();

            if(myNextId == 0)
            {
                myTimeZero = now;
            }

            const double timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now - myTimeZero).count() * 1.0e-6;

            //const double timestamp = frame.get_timestamp() * 1.0e-3;

            VideoFrame vf;
            vf.setValid(myNextId, timestamp, std::move(image));

            if(myCallback)
            {
                (*myCallback)(std::move(vf));
            }

            myNextId++;
        }
        else
        {
            std::cerr << "Invalid frame received!" << std::endl;
        }
    };

    if( myCallback )
    {
        myNextId = 0;
        mySensor.open(myProfile);
        mySensor.start(proc);
        return true;
    }
    else
    {
        return false;
    }
}

void RealsenseVideoSource::stop()
{
    mySensor.stop();
    mySensor.close();
}

int RealsenseVideoSource::getNumViews()
{
    return 1;
}

