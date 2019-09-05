
#pragma once

#include <QAbstractListModel>

class CalibrationModel : public QAbstractListModel
{
public:

    CalibrationModel(QObject* parent=nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
};

