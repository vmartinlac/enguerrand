#pragma once

#include <memory>
#include <QCloseEvent>
#include <QAction>
#include <QMainWindow>
#include "ViewerWidget.h"
#include "VideoWidget.h"
#include "Engine.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

    ~MainWindow();

protected:

    void closeEvent(QCloseEvent*) override;

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

