#include <iostream>
#include <QApplication>
#include "EngineConfig.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    if(num_args != 2)
    {
        std::cerr << "Bad command line!" << std::endl;
        exit(1);
    }

    EngineConfigPtr config(new EngineConfig());

    if( config->loadFromFile(args[1]) == false )
    {
        std::cerr << "Could not load configuration!" << std::endl;
        exit(1);
    }

    MainWindow* win = new MainWindow(config);

    win->show();

    app.exec();

    delete win;

    return 0;
}

