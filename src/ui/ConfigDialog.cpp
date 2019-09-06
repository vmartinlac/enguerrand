#include <QFileDialog>
#include <QSettings>
#include <QMessageBox>
#include <QStandardPaths>
#include <QButtonGroup>
#include "BAOdometry.h"
#include "EKFOdometry.h"
#include "ConfigDialog.h"
#include "FileVideoSource.h"

ConfigDialog::ConfigDialog(QWidget* parent) : QDialog(parent)
{
    myUI.setupUi(this);

    myOdometryCodeFactories[0] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new BAOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Bundle adjustment", 0);

    myOdometryCodeFactories[1] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new EKFOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Extended Kalman Filter", 1);

    QButtonGroup* video_group = new QButtonGroup(this);
    video_group->addButton(myUI.video_file, 0);
    video_group->addButton(myUI.video_realsense, 1);

    connect(video_group, SIGNAL(buttonClicked(int)), this, SLOT(selectVideoInput(int)));
    connect(myUI.video_file_select_path, SIGNAL(clicked()), this, SLOT(selectVideoPath()));
    connect(myUI.video_file_edit_calibrations, SIGNAL(clicked()), this, SLOT(selectCalibrationPath()));
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

void ConfigDialog::selectCalibrationPath()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select calibration file");
    if(ret.isEmpty() == false)
    {
        //myUI.video_file_path->setText(ret);
        // TODO
    }
}

void ConfigDialog::selectVideoPath()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select video file");
    if(ret.isEmpty() == false)
    {
        myUI.video_file_path->setText(ret);
    }
}

void ConfigDialog::accept()
{
    EngineConfigPtr ret = std::make_shared<EngineConfig>();

    bool ok = true;
    const char* err = "";

    if(ok)
    {
        ret->balls_histogram.reset(new Histogram());

        const QString path = QStandardPaths::locate(QStandardPaths::DataLocation, "balls_histogram.bin");

        ok = (path.isEmpty() == false) && ret->balls_histogram->load(path.toStdString());
        err = "Could not load balls histogram! Please check installation!";
    }

    if(ok)
    {
        if(myUI.video_file->isChecked())
        {
            FileVideoSourcePtr video = std::make_shared<FileVideoSource>();
            video->setFileName( myUI.video_file_path->text().toStdString() );

            ret->video_input = video;
        }
        else if(myUI.video_realsense->isChecked())
        {
        }
        else
        {
            ok = false;
            err = "Incorrect video input!";
        }
    }

    if(ok)
    {
        ret->calibration.reset(new CalibrationData());
        // TODO: set calibration.
        ok = false;
        err = "Incorrect calibration data!";
    }

    if(ok)
    {
        const int item = myUI.visual_odometry_code->currentData().toInt();
        auto it = myOdometryCodeFactories.find(item);
        if( it != myOdometryCodeFactories.end() )
        {
            ret->odometry_code = it->second(ret->calibration);
        }
        err = "Incorrect odometry code!";
        ok = false;
    }

    if(ok)
    {
        myConfig = ret;
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

EngineConfigPtr ConfigDialog::askConfig(QWidget* parent)
{
    EngineConfigPtr ret;

    ConfigDialog* dlg = new ConfigDialog(parent);

    if(dlg->exec() == QDialog::Accepted)
    {
        ret = dlg->myConfig;
    }

    delete dlg;

    return ret;
}

