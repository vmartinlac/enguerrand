
#pragma once

#include <QButtonGroup>
#include <QDialog>
#include <map>
#include "EngineConfig.h"
#include "ui_ConfigDialog.h"

class RealsenseCalibration;

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
    void selectHistogram();
    void accept() override;
    int exec() override;

protected:

    static CalibrationDataPtr convertCalibration(const RealsenseCalibration& rscalib);

protected:

    using OdometryCodeFactory = std::function<OdometryCodePtr(CalibrationDataPtr)>;

protected:

    Ui::ConfigDialog myUI;
    std::map<int,OdometryCodeFactory> myOdometryCodeFactories;
    EngineConfigPtr myConfig;
    QButtonGroup* myVideoButtonGroup;
};

