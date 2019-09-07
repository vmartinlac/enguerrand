
#pragma once

#include <QWidget>
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

protected:

    void paintEvent(QPaintEvent* ev) override;
    void updateImage();

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
    QImage myImage;
};

