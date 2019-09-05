#pragma once

#include <memory>
#include <QAction>
#include <QMainWindow>
#include "ViewerWidget.h"
#include "VideoWidget.h"
#include "Engine.h"
#include "CalibrationModel.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

    ~MainWindow();

protected slots:

    void startEngine();
    void stopEngine();
    void engineStarted();
    void engineStopped();
    void about();
    void handleFrame(EngineOutputPtr frame);

protected:

    ViewerWidget* myViewer;
    VideoWidget* myVideo;
    Engine* myEngine;
    QAction* myActionStart;
    QAction* myActionStop;
    CalibrationModel* myCalibrationModel;
};

