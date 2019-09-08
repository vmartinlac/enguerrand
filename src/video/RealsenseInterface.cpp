#include <iostream>
#include "RealsenseInterface.h"

RealsenseInterface::RealsenseInterface()
{
}
/*
    rs2::device_list devices = context.query_devices();
    for(auto item : devices)
    {
        std::cout << item.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    return 0;
*/

int RealsenseInterface::rowCount(const QModelIndex &parent) const
{
    return 0;
}

QVariant RealsenseInterface::data(const QModelIndex &index, int role) const
{
    return QVariant();
}

void RealsenseInterface::discover()
{
    rs2::device_list devices = myContext.query_devices();

    for(rs2::device device : devices)
    {
        std::vector<rs2::sensor> sensors = device.query_sensors();

        for(rs2::sensor& sensor : sensors)
        {
            std::vector<rs2::stream_profile> profiles = sensor.get_stream_profiles();

            for(size_t i=0; i<profiles.size(); i++)
            {
                if( profiles[i].stream_type() == RS2_STREAM_COLOR && profiles[i].format() == RS2_FORMAT_BGR8 && profiles[i].is<rs2::video_stream_profile>() )
                {
                    rs2::video_stream_profile video_profile = profiles[i].as<rs2::video_stream_profile>();

                    std::stringstream name;
                    name
                        << device.get_info(RS2_CAMERA_INFO_NAME) << " / "
                        << video_profile.width() << " x "
                        << video_profile.height() << " @ "
                        << profiles[i].fps() << " Hz";
                    std::cout << video_profile.stream_index() << std::endl;

                    DiscoveredSensor ds;
                    ds.name = name.str();
                    ds.device = device;
                    ds.sensor = sensor;
                    ds.stream_index = i;

                    std::cout << ds.name << std::endl;
                }
            }
        }
    }
}

