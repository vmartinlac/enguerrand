#pragma once

#include <QMainWindow>
#include "ViewerWidget.h"
#include "VideoWidget.h"
#include "EngineConfig.h"
#include "EngineThread.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(EngineConfigPtr config, QWidget* parent=nullptr);

protected slots:

    void startStopEngine(bool);
    void about();

protected:

    ViewerWidget* myViewer;
    VideoWidget* myVideo;
    EngineConfigPtr myConfig;
    EngineThread* myEngineThread;
};

