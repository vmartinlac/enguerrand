
#pragma once

#include <QWidget>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QPaintEvent>
#include "EngineOutput.h"

class VideoWidget : public QWidget
{
    Q_OBJECT

public:

    VideoWidget(QWidget* parent=nullptr);

public slots:

    void displayInputImage();
    void displayTraceImage();
    void displayEdgesImage();
    void displayDetectionImage();
    void handleFrame(EngineOutputPtr frame);
    void home();

protected:

    void paintEvent(QPaintEvent* ev) override;
    void resizeEvent(QResizeEvent* ev) override;
    void mousePressEvent(QMouseEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;

protected:

    enum DisplaySelection
    {
        DISPLAY_INPUT,
        DISPLAY_EDGES,
        DISPLAY_TRACE,
        DISPLAY_DETECTION
    };

protected:

    EngineOutputPtr myData;
    DisplaySelection myDisplaySelection;
    double myZoom;
    double myFocusX;
    double myFocusY;
    cv::Size myLastSize;
    QPoint myLastPos;
    int myMargin;
};

