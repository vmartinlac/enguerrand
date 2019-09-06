
#pragma once

#include <QDialog>
#include <map>
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
    void selectVideoPath();
    void selectCalibrationPath();
    void accept() override;

protected:

    using OdometryCodeFactory = std::function<OdometryCodePtr(CalibrationDataPtr)>;

protected:

    Ui::ConfigDialog myUI;
    std::map<int,OdometryCodeFactory> myOdometryCodeFactories;
    EngineConfigPtr myConfig;
};

