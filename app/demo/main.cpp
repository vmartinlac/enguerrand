/*
#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_context.hpp>
*/

#include <iostream>
#include <QStandardPaths>
#include <QSqlDatabase>
#include <QApplication>
#include "EngineOutput.h"
#include "EngineConfig.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
/*
    rs2::context context;
    rs2::device_list devices = context.query_devices();
    for(auto item : devices)
    {
        std::cout << item.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    return 0;
*/
    QApplication app(num_args, args);
    app.setApplicationName("enguerrand");

    qRegisterMetaType<EngineOutputPtr>();

    const QString dbpath = QStandardPaths::locate(QStandardPaths::AppDataLocation, "config.sqlite");

    if(dbpath.isEmpty())
    {
        std::cout << "Could not find config file!" << std::endl;
        exit(1);
    }

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName(dbpath);
    if( db.open() == false )
    {
        std::cout << "Could not open database!" << std::endl;
        exit(0);
    }

    MainWindow* win = new MainWindow();

    win->show();

    app.exec();

    delete win;

    return 0;
}

