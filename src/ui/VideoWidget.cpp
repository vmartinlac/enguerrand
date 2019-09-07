#include <QPainter>
#include "VideoWidget.h"

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(320, 200);
    myDisplaySelection = DISPLAY_INPUT;
    updateImage();
}

void VideoWidget::paintEvent(QPaintEvent* ev)
{
    QPainter painter(this);
    painter.fillRect(painter.viewport(), QColor(Qt::black));

    if( myImage.isNull() == false )
    {
        QImage& image = myImage;

        const int margin = 10;

        if( painter.viewport().width() > 2*margin && painter.viewport().height() > 2*margin )
        {
            QRect subview(margin, margin, painter.viewport().width()-2*margin, painter.viewport().height()-2*margin);
            QRect target;

            const double scalex = double(subview.width()) / double(image.width());
            const double scaley = double(subview.height()) / double(image.height());

            if(scalex < scaley)
            {
                const int target_height = image.height()*subview.width()/image.width();
                target = QRect(subview.x(), subview.y() + subview.height()/2-target_height/2, subview.width(), target_height);
            }
            else
            {
                const int target_width = image.width()*subview.height()/image.height();
                target = QRect(subview.x() + subview.width()/2-target_width/2, subview.y(), target_width, subview.height());
            }

            const QRect source(0, 0, image.width(), image.height());
            painter.drawImage(target, image, source);
        }
    }
}

void VideoWidget::handleFrame(EngineOutputPtr frame)
{
    myData = frame;
    updateImage();
}

void VideoWidget::displayInputImage()
{
    myDisplaySelection = DISPLAY_INPUT;
    updateImage();
}

void VideoWidget::displayEdgesImage()
{
    myDisplaySelection = DISPLAY_EDGES;
    updateImage();
}

void VideoWidget::displayTraceImage()
{
    myDisplaySelection = DISPLAY_TRACE;
    updateImage();
}

void VideoWidget::displayDetectionImage()
{
    myDisplaySelection = DISPLAY_DETECTION;
    updateImage();
}

void VideoWidget::updateImage()
{
    myImage = QImage();

    if( myData )
    {
        if( myDisplaySelection == DISPLAY_INPUT )
        {
            cv::Mat3b imagecv = myData->input_image;

            myImage = QImage(
                imagecv.data,
                imagecv.cols,
                imagecv.rows,
                imagecv.step,
                QImage::Format_RGB888).rgbSwapped();
        }
        else if( myDisplaySelection == DISPLAY_EDGES )
        {
            cv::Mat1b imagecv = myData->edges_image;

            myImage = QImage(
                imagecv.data,
                imagecv.cols,
                imagecv.rows,
                imagecv.step,
                QImage::Format_Grayscale8).copy();
        }
        else if( myDisplaySelection == DISPLAY_TRACE )
        {
            cv::Mat3b imagecv = myData->traces_image;

            myImage = QImage(
                imagecv.data,
                imagecv.cols,
                imagecv.rows,
                imagecv.step,
                QImage::Format_RGB888).rgbSwapped();
        }
        else
        {
            std::cerr << "Internal error" << std::endl;
            exit(1);
        }
    }

    update();
}

