#pragma once

#include <QAction>
#include <QMainWindow>
#include "ViewerWidget.h"
#include "VideoWidget.h"
#include "Engine.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(EngineConfigPtr config, QWidget* parent=nullptr);
    ~MainWindow();

protected slots:

    void startEngine();
    void stopEngine();
    void engineStarted();
    void engineStopped();
    void about();

protected:

    ViewerWidget* myViewer;
    VideoWidget* myVideo;
    Engine* myEngine;
    QAction* myActionStart;
    QAction* myActionStop;
};

