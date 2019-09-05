
#pragma once

#include <QDialog>
#include "CalibrationModel.h"

class CalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    CalibrationDialog(CalibrationModel* model, QWidget* parent=nullptr);

protected:

    CalibrationModel* myModel;
};

