#include <iostream>
#include <QApplication>
#include "EngineConfig.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* win = new MainWindow();

    win->show();

    app.exec();

    delete win;

    return 0;
}

