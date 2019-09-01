#include <QFileDialog>
#include <QButtonGroup>
#include "BAOdometry.h"
#include "EKFOdometry.h"
#include "ConfigDialog.h"

ConfigDialog::ConfigDialog(QWidget* parent) : QDialog(parent)
{
    myUI.setupUi(this);

    myOdometryCodeFactories.emplace_back( [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new BAOdometry(calib)); } );
    myUI.visual_odometry_code->addItem("Bundle adjustment");

    myOdometryCodeFactories.emplace_back( [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new EKFOdometry(calib)); } );
    myUI.visual_odometry_code->addItem("Extended Kalman Filter");

    QButtonGroup* video_group = new QButtonGroup(this);
    video_group->addButton(myUI.video_file, 0);
    video_group->addButton(myUI.video_realsense, 1);

    connect(video_group, SIGNAL(buttonClicked(int)), this, SLOT(selectVideoInput(int)));
    connect(myUI.video_file_select_path, SIGNAL(clicked()), this, SLOT(selectPath()));
    connect(myUI.video_file_edit_calibrations, SIGNAL(clicked()), this, SLOT(editCalibrations()));
}

void ConfigDialog::selectVideoInput(int btn)
{
    if(btn == 0)
    {
        myUI.video_file_path->setEnabled(true);
        myUI.video_file_select_path->setEnabled(true);
        myUI.video_file_calibration->setEnabled(true);
        myUI.video_file_edit_calibrations->setEnabled(true);
        myUI.video_realsense_camera->setEnabled(false);
    }
    else if(btn == 1)
    {
        myUI.video_file_path->setEnabled(false);
        myUI.video_file_select_path->setEnabled(false);
        myUI.video_file_calibration->setEnabled(false);
        myUI.video_file_edit_calibrations->setEnabled(false);
        myUI.video_realsense_camera->setEnabled(true);
    }
    else
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }
}

void ConfigDialog::editCalibrations()
{
    std::cout << "TODO" << std::endl;
}

void ConfigDialog::selectPath()
{
    QString ret = QFileDialog::getOpenFileName(this, "Select video file");
    if(ret.isEmpty() == false)
    {
        myUI.video_file_path->setText(ret);
    }
}

EngineConfigPtr ConfigDialog::askConfig(QWidget* parent)
{
    EngineConfigPtr ret;

    ConfigDialog* dlg = new ConfigDialog(parent);

    if(dlg->exec() == QDialog::Accepted)
    {
        ret = std::make_shared<EngineConfig>();

        // TODO
        if( ret->loadFromFile("/home/victor/developpement/enguerrand/enguerrand/config/config_faber.json") == false ) ret.reset();
        //
    }

    delete dlg;

    return ret;
}

