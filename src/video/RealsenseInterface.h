#pragma once

#include <QAbstractListModel>
#include <vector>
#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_context.hpp>
#include "RealsenseVideoSource.h"

class RealsenseInterface : public QAbstractListModel
{
    Q_OBJECT

public:

    RealsenseInterface();
    ~RealsenseInterface();

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    static RealsenseInterface* instance();

    RealsenseVideoSourcePtr createVideoSource(const QModelIndex& index);

    QModelIndex index(int row, int column, const QModelIndex& parent=QModelIndex()) const override;

public slots:

    void discover();

protected:

    struct DiscoveredSensor
    {
        QString name;
        rs2::sensor sensor;
        rs2::stream_profile profile;
    };

protected:

    rs2::context myContext;
    std::vector<DiscoveredSensor> mySensors;
    static RealsenseInterface* myInstance;
};

