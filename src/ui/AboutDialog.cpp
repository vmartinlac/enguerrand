#include "BuildInfo.h"
#include "AboutDialog.h"

AboutDialog::AboutDialog(QWidget* parent) : QDialog(parent)
{
   myUI.setupUi(this);

   const char* pattern =
    "<html></head><body>"
    "<p>Enguerrand demonstrates three classical approaches to visual odometry: bundle adjustment, kalman filter and particle filter.</p>"
    "<p>Enguerrand was writen from may 2019 to november 2019 by Victor MARTIN-LAC.</p>"
    "<p>Icons come from <a href='https://icons8.com'>https://icons8.com/</a>.</p>"
    "</body></html>";

    const QString text = QString(pattern).arg(10);

    myUI.content->setText(text);
    myUI.version_label->setText(ENGUERRAND_VERSION_STRING);
    myUI.build_type_label->setText(ENGUERRAND_BUILD_TYPE);
    myUI.compiler_label->setText(ENGUERRAND_COMPILER);
    myUI.platform_label->setText(ENGUERRAND_SYSTEM_NAME);
}

