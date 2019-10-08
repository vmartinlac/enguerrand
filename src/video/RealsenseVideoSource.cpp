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

bool RealsenseVideoSource::getCalibration(RealsenseCalibration& calibration) const
{
    rs2::video_stream_profile profile(myProfile);
    rs2_intrinsics intrinsics = profile.get_intrinsics();

    const bool ok = (intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY || intrinsics.model == RS2_DISTORTION_NONE);

    if(ok)
    {
        calibration.image_size.width = intrinsics.width;
        calibration.image_size.height = intrinsics.height;

        calibration.calibration_matrix <<
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0;

        if(intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
        {
            calibration.distortion_coefficients.resize(5);
            std::copy(intrinsics.coeffs, intrinsics.coeffs+5, calibration.distortion_coefficients.begin());
        }
        else
        {
            calibration.distortion_coefficients.clear();
        }
    }

    return ok;
}

