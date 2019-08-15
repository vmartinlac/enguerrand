#pragma once

#include <QMainWindow>
#include "ViewerWidget.h"
#include "VideoWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected slots:

    void startStopEngine(bool);
    void about();

protected:

    ViewerWidget* myViewer;
    VideoWidget* myVideo;
};

