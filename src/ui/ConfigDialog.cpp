#include <QFileDialog>
#include <QSettings>
#include <QMessageBox>
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

    myVideoButtonGroup = new QButtonGroup(this);
    myVideoButtonGroup->addButton(myUI.video_file, 0);
    myVideoButtonGroup->addButton(myUI.video_realsense, 1);

    connect(myVideoButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(selectVideoInput(int)));
    connect(myUI.video_file_select_video, SIGNAL(clicked()), this, SLOT(selectVideoPath()));
    connect(myUI.video_file_select_calibration, SIGNAL(clicked()), this, SLOT(selectCalibrationPath()));
    connect(myUI.histogram_select, SIGNAL(clicked()), this, SLOT(selectHistogram()));

    selectVideoInput(0);
}

void ConfigDialog::selectVideoInput(int btn)
{
    if(btn == 0)
    {
        myUI.video_file_path->setEnabled(true);
        myUI.video_file_select_video->setEnabled(true);
        myUI.video_file_calibration->setEnabled(true);
        myUI.video_file_select_calibration->setEnabled(true);
        myUI.video_realsense_camera->setEnabled(false);
    }
    else if(btn == 1)
    {
        myUI.video_file_path->setEnabled(false);
        myUI.video_file_select_video->setEnabled(false);
        myUI.video_file_calibration->setEnabled(false);
        myUI.video_file_select_calibration->setEnabled(false);
        myUI.video_realsense_camera->setEnabled(true);
    }
    else
    {
        std::cerr << "Internal error" << std::endl;
        exit(1);
    }
}

void ConfigDialog::selectHistogram()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select histogram file");

    if(ret.isEmpty() == false)
    {
        myUI.histogram_path->setText(ret);
    }
}

void ConfigDialog::selectCalibrationPath()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select calibration file");

    if(ret.isEmpty() == false)
    {
        myUI.video_file_calibration->setText(ret);
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
        const QString path = myUI.histogram_path->text();
        ok = ( path.isEmpty() == false );
        err = "Please set histogram path!";

        if(ok)
        {
            ret->balls_histogram = std::make_shared<Histogram>();
            ok = ret->balls_histogram->load(path.toStdString());
            err = "Could not load histogram!";
        }
    }

    if(ok)
    {
        if(myUI.video_file->isChecked())
        {
            const std::string filename = myUI.video_file_path->text().toStdString();

            ok = ( filename.empty() == false );
            err = "Please set path to video!";

            if(ok)
            {
                FileVideoSourcePtr video = std::make_shared<FileVideoSource>();
                video->setFileName(filename);
                ret->video_input = video;
            }
        }
        else if(myUI.video_realsense->isChecked())
        {
            ok = false; // TODO
            err = "Realsense not implemented yet!";
        }
        else
        {
            ok = false;
            err = "Incorrect video input!";
        }
    }

    if(ok)
    {
        const std::string path = myUI.video_file_calibration->text().toStdString();

        ok = (path.empty() == false);
        err = "Please set path to calibration data!";

        if(ok)
        {
            ret->calibration.reset(new CalibrationData());
            ok = ret->calibration->loadFromFile(path);
            err = "Incorrect calibration data!";
        }
    }

    if(ok)
    {
        const int item = myUI.visual_odometry_code->currentData().toInt();
        auto it = myOdometryCodeFactories.find(item);
        if( it == myOdometryCodeFactories.end() )
        {
            err = "Incorrect odometry code!";
            ok = false;
        }
        else
        {
            ret->odometry_code = it->second(ret->calibration);
        }
    }

    if(ok)
    {
        QSettings s;

        s.beginGroup("ConfigDialog");
        s.setValue("video_file_path", myUI.video_file_path->text());
        s.setValue("video_file_calibration", myUI.video_file_calibration->text());
        s.setValue("video_realsense_camera", myUI.video_realsense_camera->currentIndex());
        s.setValue("video", myVideoButtonGroup->checkedId());
        s.setValue("visual_odometry_code", myUI.visual_odometry_code->currentIndex());
        s.setValue("histogram_path", myUI.histogram_path->text());
        s.endGroup();

        s.sync();

        myConfig = ret;
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

int ConfigDialog::exec()
{
    QSettings s;

    s.beginGroup("ConfigDialog");
    myUI.video_file_path->setText(s.value("video_file_path", QString()).toString());
    myUI.video_file_calibration->setText(s.value("video_file_calibration", QString()).toString());
    myUI.video_realsense_camera->setCurrentIndex(s.value("video_realsense_camera", 0).toInt());
    myUI.visual_odometry_code->setCurrentIndex(s.value("visual_odometry_code", 0).toInt());
    QAbstractButton* btn = myVideoButtonGroup->button( s.value("video", 0).toInt() );
    if(btn) btn->setChecked(true);
    myUI.histogram_path->setText( s.value("histogram_path", QString()).toString() );
    s.endGroup();

    return QDialog::exec();
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

