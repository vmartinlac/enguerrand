#include <QFormLayout>
#include <QLineEdit>
#include <iostream>
#include <QKeySequence>
#include <QMessageBox>
#include <QApplication>
#include <QToolBar>
#include <QLabel>
#include <QSplitter>
#include <QAction>
#include "MainWindow.h"
#include "ViewerWidget.h"

MainWindow::MainWindow(EngineConfigPtr config, QWidget* parent) : QMainWindow(parent)
{
    myConfig = config;
    myEngineThread = new EngineThread(config);

    QToolBar* tb = addToolBar("Tools");
    QAction* action_quit = tb->addAction("Quit");
    QAction* action_run = tb->addAction("Run");
    QAction* action_about = tb->addAction("About");

    action_run->setShortcut(QKeySequence("Ctrl+R"));
    action_about->setShortcut(QKeySequence("F1"));
    action_quit->setShortcut(QKeySequence("Ctrl+Q"));
    action_run->setCheckable(true);

    myVideo = new VideoWidget();

    myViewer = new ViewerWidget();

    QSplitter* splitter = new QSplitter();
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(myVideo);
    splitter->addWidget(myViewer);

    setCentralWidget(splitter);
    setWindowTitle("Enguerrand");

    connect(action_about, SIGNAL(triggered()), this, SLOT(about()));
    connect(action_run, SIGNAL(toggled(bool)), this, SLOT(startStopEngine(bool)));
    connect(action_quit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor MARTIN-LAC 2019");
}

void MainWindow::startStopEngine(bool value)
{
    std::cout << "TODO" << std::endl;
}

