#pragma once

#include <QAbstractListModel>
#include <vector>
#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_context.hpp>

class RealsenseInterface : public QAbstractListModel
{
public:

    RealsenseInterface();

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    void discover();

protected:

    struct DiscoveredSensor
    {
        std::string name;
        rs2::device device;
        rs2::sensor sensor;
        int stream_index;
    };

protected:

    rs2::context myContext;
    std::vector<DiscoveredSensor> mySensors;
};

