#include <iostream>
#include "RealsenseInterface.h"

RealsenseInterface* RealsenseInterface::myInstance = nullptr;

RealsenseInterface* RealsenseInterface::instance()
{
    return myInstance;
}

RealsenseInterface::RealsenseInterface()
{
    if(myInstance != nullptr)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }
    myInstance = this;
}

RealsenseInterface::~RealsenseInterface()
{
    if(myInstance != this)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }
    myInstance = nullptr;
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
    return mySensors.size();
}

QVariant RealsenseInterface::data(const QModelIndex &index, int role) const
{
    QVariant ret;

    if(role == Qt::DisplayRole && index.row() < mySensors.size())
    {
        ret = QString(mySensors[index.row()].name);
    }

    return ret;
}

void RealsenseInterface::discover()
{
    std::vector<DiscoveredSensor> new_sensors;

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

                    const QString name =
                        QString("%1 / %2 x %3 @ %4 Hz")
                        .arg(device.get_info(RS2_CAMERA_INFO_NAME))
                        .arg(video_profile.width())
                        .arg(video_profile.height())
                        .arg(profiles[i].fps());

                    /*
                    std::stringstream name;
                    name
                        << device.get_info(RS2_CAMERA_INFO_NAME) << " / "
                        << video_profile.width() << " x "
                        << video_profile.height() << " @ "
                        << profiles[i].fps() << " Hz";
                    */

                    DiscoveredSensor ds;
                    ds.name = name;
                    ds.sensor = sensor;
                    ds.profile = video_profile;

                    new_sensors.push_back(ds);
                }
            }
        }
    }

    if( mySensors.empty() == false )
    {
        beginRemoveRows(QModelIndex(), 0, mySensors.size()-1);
        mySensors.clear();
        endRemoveRows();
    }

    if( new_sensors.empty() == false )
    {
        beginInsertRows(QModelIndex(), 0, new_sensors.size()-1);
        mySensors.swap(new_sensors);
        endInsertRows();
    }
}

RealsenseVideoSourcePtr RealsenseInterface::createVideoSource(const QModelIndex& index)
{
    RealsenseVideoSourcePtr ret;

    if(index.isValid())
    {
        DiscoveredSensor& ds = mySensors[index.internalId()];
        ret.reset(new RealsenseVideoSource(ds.sensor, ds.profile));
    }

    return ret;
}

QModelIndex RealsenseInterface::index(int row, int column, const QModelIndex& parent) const
{
    QModelIndex ret;

    if(column == 0 && 0 <= row && row < mySensors.size() && parent.isValid() == false)
    {
        ret = createIndex(row, column, row);
    }

    return ret;
}

