
#pragma once

#include <opencv2/core.hpp>
#include <QWidget>
#include <QPaintEvent>

class VideoWidget : public QWidget
{
public:

    VideoWidget(QWidget* parent=nullptr);

protected:

    void paintEvent(QPaintEvent* ev) override;

protected:

    cv::Mat3b myImage;
};

