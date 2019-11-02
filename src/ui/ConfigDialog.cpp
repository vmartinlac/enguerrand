#include <thread>
#include <QFileDialog>
#include <QSettings>
#include <QMessageBox>
#include "ConfigDialog.h"
#include "FileVideoSource.h"
#include "RealsenseInterface.h"
#include "NullOdometry.h"
#include "PFOdometry.h"
#include "BAOdometry.h"
#include "EKFOdometry.h"
#include "AlignmentOdometry.h"
#include "ObservationValidatorSVM.h"
#include "ObservationValidatorSimple.h"

ConfigDialog::ConfigDialog(QWidget* parent) : QDialog(parent)
{
    myUI.setupUi(this);

    myOdometryCodeFactories[0] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new BAOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Bundle adjustment", 0);

    myOdometryCodeFactories[1] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new EKFOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Extended Kalman Filter", 1);

    myOdometryCodeFactories[2] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new PFOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Particle filter", 2);

    myOdometryCodeFactories[3] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new AlignmentOdometry(calib)); };
    myUI.visual_odometry_code->addItem("Alignment", 3);

    myOdometryCodeFactories[4] = [] (CalibrationDataPtr calib) -> OdometryCodePtr { return OdometryCodePtr(new NullOdometry()); };
    myUI.visual_odometry_code->addItem("None", 4);


    myObservationValidatorFactories[0] = [] () -> ObservationValidatorPtr { return std::make_shared<ObservationValidatorSimple>(); };
    myUI.observation_validator->addItem("Distance to reference histogram", 0);

    myObservationValidatorFactories[1] = [] () -> ObservationValidatorPtr { return std::make_shared<ObservationValidatorSVM>(); };
    myUI.observation_validator->addItem("Support Vector Machine", 1);


    myVideoButtonGroup = new QButtonGroup(this);
    myVideoButtonGroup->addButton(myUI.video_file, 0);
    myVideoButtonGroup->addButton(myUI.video_realsense, 1);

    RealsenseInterface::instance()->discover();
    myUI.video_realsense_camera->setModel(RealsenseInterface::instance());

    myUI.video_file->setChecked(true);
    if(RealsenseInterface::instance()->rowCount() == 0)
    {
        myUI.video_realsense->setEnabled(false);
        myUI.video_realsense_camera->setEnabled(false);
        myUI.camera_label->setEnabled(false);
    }

#ifndef WITH_CUDA
    myUI.use_gpu_label->setEnabled(false);
    myUI.use_gpu->setChecked(false);
    myUI.use_gpu->setEnabled(false);
#endif

    connect(myVideoButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(selectVideoInput(int)));
    connect(myUI.video_file_select_video, SIGNAL(clicked()), this, SLOT(selectVideoPath()));
    connect(myUI.video_file_select_calibration, SIGNAL(clicked()), this, SLOT(selectCalibrationPath()));
    connect(myUI.histogram_select, SIGNAL(clicked()), this, SLOT(selectObservationValidatorData()));

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

void ConfigDialog::selectObservationValidatorData()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select data file", myUI.observation_validator_data->text());

    if(ret.isEmpty() == false)
    {
        myUI.observation_validator_data->setText(ret);
    }
}

void ConfigDialog::selectCalibrationPath()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select calibration file", myUI.video_file_calibration->text());

    if(ret.isEmpty() == false)
    {
        myUI.video_file_calibration->setText(ret);
    }
}

void ConfigDialog::selectVideoPath()
{
    const QString ret = QFileDialog::getOpenFileName(this, "Select video file", myUI.video_file_path->text());

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
#ifdef WITH_CUDA
        if( myUI.use_gpu->isChecked() )
        {
            ret->edge_detector = EdgeDetection::createEdgeDetectionGPU();
        }
        else
#endif
        {
            ret->edge_detector = EdgeDetection::createEdgeDetectionCPU();
        }

        ok = bool(ret->edge_detector);
        err = "Could not create edge detector!";
    }

    if(ok)
    {
        const int item = myUI.observation_validator->currentData().toInt();
        auto it = myObservationValidatorFactories.find(item);
        if( it == myObservationValidatorFactories.end() )
        {
            err = "Incorrect observation validator!";
            ok = false;
        }
        else
        {
            ret->observation_validator = it->second();
        }
    }

    if(ok)
    {
        const QString path = myUI.observation_validator_data->text();
        ok = ( path.isEmpty() == false );
        err = "Please set path to observation validator data!";

        if(ok)
        {
            ok = ret->observation_validator->load(path.toStdString());
            err = "Could not load observation validator data!";
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

                const std::string path = myUI.video_file_calibration->text().toStdString();

                ok = (path.empty() == false);
                err = "Please set path to calibration data!";

                if(ok)
                {
                    ret->calibration.reset(new CalibrationData());
                    ok = ret->calibration->loadCamerasFromFile(path);
                    err = "Incorrect calibration data!";
                }
            }
        }
        else if(myUI.video_realsense->isChecked())
        {
            RealsenseInterface* intf = RealsenseInterface::instance();
            RealsenseVideoSourcePtr video = intf->createVideoSource( intf->index( myUI.video_realsense_camera->currentIndex(), 0) );
            ok = bool(video);
            err = "Please select valid realsense camera!";

            if(ok)
            {
                ret->video_input = video;

                /*
                auto proc = [] (VideoFrame&& vf)
                {
                    std::cerr << "Frame! " << vf.getTimestamp() << std::endl;
                };
                video->setCallback(proc);
                video->start();
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                video->stop();
                */

                RealsenseCalibration rscalib;

                ok = bool(video->getCalibration(rscalib));
                err = "Could not retrieve calibration data from camera!";

                if(ok)
                {
                    ret->calibration = convertCalibration(rscalib);

                    ok = bool(ret->calibration);
                    err = "Could not decode calibration data from camera!";
                }
            }
        }
        else
        {
            ok = false;
            err = "Incorrect video input!";
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
        //ret->calibration->dump();

        QSettings s;

        s.beginGroup("ConfigDialog");
        s.setValue("video_file_path", myUI.video_file_path->text());
        s.setValue("video_file_calibration", myUI.video_file_calibration->text());
        s.setValue("video_realsense_camera", myUI.video_realsense_camera->currentIndex());
        s.setValue("video", myVideoButtonGroup->checkedId());
        s.setValue("visual_odometry_code", myUI.visual_odometry_code->currentIndex());
        s.setValue("observation_validator", myUI.observation_validator->currentIndex());
        s.setValue("observation_validator_data", myUI.observation_validator_data->text());
        s.setValue("use_gpu", myUI.use_gpu->isChecked());
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
    myUI.observation_validator->setCurrentIndex(s.value("observation_validator", 0).toInt());
    const int btn = s.value("video", 0).toInt();
    if(btn == 0 || btn == 1) selectVideoInput(btn);
    QAbstractButton* btn2 = myVideoButtonGroup->button( s.value("video", 0).toInt() );
    if(btn2) btn2->setChecked(true);
    myUI.observation_validator_data->setText( s.value("observation_validator_data", QString()).toString() );
#if WITH_CUDA
    myUI.use_gpu->setChecked( s.value("use_gpu", false).toBool() );
#else
    myUI.use_gpu->setChecked(false);
#endif
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

CalibrationDataPtr ConfigDialog::convertCalibration(const RealsenseCalibration& rscalib)
{
    CalibrationDataPtr ret = std::make_shared<CalibrationData>();

    ret->cameras.resize(1);
    CameraCalibrationData& cam = ret->cameras.front();
    
    cam.image_size = rscalib.image_size;

    cam.calibration_matrix = rscalib.calibration_matrix;

    const double fx = rscalib.calibration_matrix(0,0);
    const double fy = rscalib.calibration_matrix(1,1);
    const double cx = rscalib.calibration_matrix(0,2);
    const double cy = rscalib.calibration_matrix(1,2);

    cam.inverse_calibration_matrix <<
        1.0/fx, 0.0, -cx/fx,
        0.0, 1.0/fy, -cy/fy,
        0.0, 0.0, 1.0;

    cam.distortion_coefficients = rscalib.distortion_coefficients;

    cam.camera_to_robot = Sophus::SE3d(); // identity.

    return ret;
}

