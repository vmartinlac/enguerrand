#include <QPainter>
#include "VideoWidget.h"

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    myMargin = 10;
    setMinimumSize(320, 200);
    myDisplaySelection = DISPLAY_INPUT;
    myZoom = 1.0;
    myFocusX = 0.0;
    myFocusY = 0.0;
    myLastSize = cv::Size(0,0);
}

void VideoWidget::paintEvent(QPaintEvent* ev)
{
    QPainter painter(this);
    painter.fillRect(painter.viewport(), QColor(Qt::black));

    if( myData && painter.viewport().width() > 2*myMargin && painter.viewport().height() > 2*myMargin )
    {
        //QRect subview(myMargin, myMargin, painter.viewport().width()-2*myMargin, painter.viewport().height()-2*myMargin);
        //painter.setClipRect(subview);

        const auto vp = painter.viewport();

        if( myLastSize != myData->input_image.size() )
        {
            myLastSize = myData->input_image.size();
            myFocusX = myLastSize.width*0.5;
            myFocusY = myLastSize.height*0.5;

            const double scalex = double(vp.width() - 2*myMargin) / double(myLastSize.width);
            const double scaley = double(vp.height() - 2*myMargin) / double(myLastSize.height);
            myZoom = std::min<double>(scalex, scaley);
        }

        QImage image;

        if(myDisplaySelection == DISPLAY_INPUT)
        {
            cv::Mat3b imagecv = myData->input_image;
            image = QImage(imagecv.data, imagecv.cols, imagecv.rows, imagecv.step, QImage::Format_RGB888);
        }
        else if(myDisplaySelection == DISPLAY_EDGES)
        {
            cv::Mat1b imagecv = myData->edges_image;
            image = QImage(imagecv.data, imagecv.cols, imagecv.rows, imagecv.step, QImage::Format_Grayscale8);
        }
        else if(myDisplaySelection == DISPLAY_TRACE)
        {
            cv::Mat3b imagecv = myData->traces_image;
            image = QImage(imagecv.data, imagecv.cols, imagecv.rows, imagecv.step, QImage::Format_RGB888);
        }
        else if(myDisplaySelection == DISPLAY_DETECTION)
        {
            cv::Mat3b imagecv = myData->detection_image;
            image = QImage(imagecv.data, imagecv.cols, imagecv.rows, imagecv.step, QImage::Format_RGB888);
        }
        else
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        QTransform tf =
            QTransform::fromTranslate(-myFocusX, -myFocusY) *
            QTransform::fromScale(myZoom, myZoom) *
            QTransform::fromTranslate(vp.width()/2, vp.height()/2);

        painter.setTransform(tf);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::SmoothPixmapTransform);

        painter.drawImage(QPoint(0,0), image);
    }

    ev->accept();
}

void VideoWidget::mousePressEvent(QMouseEvent* ev)
{
    myLastPos = ev->globalPos();
    ev->accept();
}

void VideoWidget::mouseMoveEvent(QMouseEvent* ev)
{
    const double dx = ev->globalPos().x() - myLastPos.x();
    const double dy = ev->globalPos().y() - myLastPos.y();

    myFocusX -= dx/myZoom;
    myFocusY -= dy/myZoom;

    myLastPos = ev->globalPos();
    ev->accept();
    update();
}

void VideoWidget::wheelEvent(QWheelEvent* ev)
{
    const double gamma = 1.0e-3;
    myZoom *= std::exp(gamma*ev->angleDelta().y());
    ev->accept();
    update();
}

void VideoWidget::home()
{
    if( myData )
    {
        myLastSize = myData->input_image.size();
        myFocusX = myLastSize.width*0.5;
        myFocusY = myLastSize.height*0.5;

        const double scalex = double(width() - 2*myMargin) / double(myLastSize.width);
        const double scaley = double(height() - 2*myMargin) / double(myLastSize.height);
        myZoom = std::min<double>(scalex, scaley);
    }
    else
    {
        myLastSize = cv::Size(0,0);
        myFocusX = 0.0;
        myFocusY = 0.0;
        myZoom = 1.0;
    }

    update();
}

void VideoWidget::resizeEvent(QResizeEvent* ev)
{
    //myZoom *=
    //ev->accept();
    //update();
    QWidget::resizeEvent(ev);
}

void VideoWidget::handleFrame(EngineOutputPtr frame)
{
    myData = frame;
    update();
}

void VideoWidget::displayInputImage()
{
    myDisplaySelection = DISPLAY_INPUT;
    update();
}

void VideoWidget::displayEdgesImage()
{
    myDisplaySelection = DISPLAY_EDGES;
    update();
}

void VideoWidget::displayTraceImage()
{
    myDisplaySelection = DISPLAY_TRACE;
    update();
}

void VideoWidget::displayDetectionImage()
{
    myDisplaySelection = DISPLAY_DETECTION;
    update();
}

