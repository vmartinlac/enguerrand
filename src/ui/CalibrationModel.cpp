#include "CalibrationModel.h"

CalibrationModel::CalibrationModel(QObject* parent) : QAbstractListModel(parent)
{
}

int CalibrationModel::rowCount(const QModelIndex &parent) const
{
    return 1;
}

QVariant CalibrationModel::data(const QModelIndex &index, int role) const
{
    if(index.isValid() && index.row() == 0 && index.column() == 0 && index.parent().isValid() == false)
    {
        return QString("hello");
    }
    else
    {
        return QVariant();
    }
}

QVariant CalibrationModel::headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const
{
    return QString("world");
}

