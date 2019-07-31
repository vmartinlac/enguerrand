#include <QToolBar>
#include <QSplitter>
#include <QAction>
#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    QToolBar* tb = addToolBar("Tools");
    setWindowTitle("Enguerrand");
}

