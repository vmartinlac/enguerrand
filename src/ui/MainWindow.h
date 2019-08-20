#pragma once

#include <memory>
#include <QAction>
#include <QMainWindow>
#include "ViewerWidget.h"
#include "DefaultEngineListener.h"
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
    void handleFrame();

protected:

    ViewerWidget* myViewer;
    VideoWidget* myVideo;
    Engine* myEngine;
    std::unique_ptr<DefaultEngineListener> myListener;
    QAction* myActionStart;
    QAction* myActionStop;
};

