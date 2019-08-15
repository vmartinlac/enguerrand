#include <QPainter>
#include "VideoWidget.h"

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(320, 200);

    //
    myImage.create(480, 640);
    std::fill(myImage.begin(), myImage.end(), cv::Vec3b(0, 255, 0));
}

void VideoWidget::paintEvent(QPaintEvent* ev)
{
    QPainter painter(this);
    painter.fillRect(painter.viewport(), QColor(Qt::black));

    if( myImage.data )
    {
        QImage image(
            myImage.data,
            myImage.cols,
            myImage.rows,
            myImage.step,
            QImage::Format_RGB888);

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

