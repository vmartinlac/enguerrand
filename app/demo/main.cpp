#include <iostream>
#include <QApplication>
#include "EngineOutput.h"
#include "RealsenseInterface.h"
#include "EngineConfig.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);
    app.setApplicationName("enguerrand");

    qRegisterMetaType<EngineOutputPtr>();
    qRegisterMetaType<EngineConfigPtr>();

    RealsenseInterface* realsense_interface = new RealsenseInterface();
    //realsense_interface->discover();

    MainWindow* win = new MainWindow();

    win->showMaximized();
    win->show();

    app.exec();

    delete win;

    delete realsense_interface;

    return 0;
}

