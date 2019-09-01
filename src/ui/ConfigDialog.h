
#pragma once

#include <QDialog>
#include "EngineConfig.h"
#include "ui_ConfigDialog.h"

class ConfigDialog : public QDialog
{

Q_OBJECT

public:

    ConfigDialog(QWidget* parent=nullptr);

    static EngineConfigPtr askConfig(QWidget* parent=nullptr);

protected slots:

    void selectVideoInput(int);
    void editCalibrations();
    void selectPath();

protected:

    using OdometryCodeFactory = std::function<OdometryCodePtr(CalibrationDataPtr)>;

protected:

    Ui::ConfigDialog myUI;
    std::vector<OdometryCodeFactory> myOdometryCodeFactories;
};

