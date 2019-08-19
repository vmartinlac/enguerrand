#include <iostream>
#include <QKeySequence>
#include <QMessageBox>
#include <QApplication>
#include <QToolBar>
#include <QLabel>
#include <QSplitter>
#include "MainWindow.h"
#include "ViewerWidget.h"

MainWindow::MainWindow(EngineConfigPtr config, QWidget* parent) : QMainWindow(parent)
{
    myEngine = new Engine(this);
    myEngine->setConfig(config);

    QToolBar* tb = addToolBar("Tools");
    QAction* action_quit = tb->addAction("Quit");
    QAction* action_start = tb->addAction("Start");
    QAction* action_stop = tb->addAction("Stop");
    QAction* action_about = tb->addAction("About");

    myActionStart = action_start;
    myActionStop = action_stop;

    //action_start->setShortcut(QKeySequence("Ctrl+R"));
    action_about->setShortcut(QKeySequence("F1"));
    action_quit->setShortcut(QKeySequence("Ctrl+Q"));
    action_stop->setEnabled(false);

    myVideo = new VideoWidget();

    myViewer = new ViewerWidget();

    QSplitter* splitter = new QSplitter();
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(myVideo);
    splitter->addWidget(myViewer);

    setCentralWidget(splitter);
    setWindowTitle("Enguerrand");

    connect(action_about, SIGNAL(triggered()), this, SLOT(about()));
    connect(action_start, SIGNAL(triggered()), this, SLOT(startEngine()));
    connect(action_stop, SIGNAL(triggered()), this, SLOT(stopEngine()));
    connect(action_quit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
    connect(myEngine, SIGNAL(started()), this, SLOT(engineStarted()));
    connect(myEngine, SIGNAL(finished()), this, SLOT(engineStopped()));
}

MainWindow::~MainWindow()
{
    if( myEngine->isRunning())
    {
        myEngine->requestInterruption();
        myEngine->wait();
    }
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor MARTIN-LAC 2019");
}

void MainWindow::startEngine()
{
    myActionStart->setEnabled(false);
    myActionStop->setEnabled(false);
    myEngine->start();
}

void MainWindow::stopEngine()
{
    myActionStart->setEnabled(false);
    myActionStop->setEnabled(false);
    myEngine->requestInterruption();
}

void MainWindow::engineStarted()
{
    myActionStart->setEnabled(false);
    myActionStop->setEnabled(true);
}

void MainWindow::engineStopped()
{
    myActionStart->setEnabled(true);
    myActionStop->setEnabled(false);
}

