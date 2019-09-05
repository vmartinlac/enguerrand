#include <iostream>
#include <QKeySequence>
#include <QMessageBox>
#include <QApplication>
#include <QToolBar>
#include <QLabel>
#include <QSplitter>
#include "MainWindow.h"
#include "ViewerWidget.h"
#include "ConfigDialog.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    myEngine = new Engine(this);

    myCalibrationModel = new CalibrationModel(this);
    myCalibrationModel->setHeaderData(0, Qt::Horizontal, "Name");
    myCalibrationModel->setHeaderData(1, Qt::Horizontal, "fx");
    myCalibrationModel->setHeaderData(2, Qt::Horizontal, "fy");
    myCalibrationModel->setHeaderData(3, Qt::Horizontal, "cx");
    myCalibrationModel->setHeaderData(4, Qt::Horizontal, "cy");
    myCalibrationModel->setHeaderData(5, Qt::Horizontal, "k1");
    myCalibrationModel->setHeaderData(6, Qt::Horizontal, "k2");
    myCalibrationModel->setHeaderData(7, Qt::Horizontal, "p1");
    myCalibrationModel->setHeaderData(8, Qt::Horizontal, "p2");
    myCalibrationModel->setHeaderData(9, Qt::Horizontal, "k3");

    connect(myEngine, SIGNAL(newFrame(EngineOutputPtr)), this, SLOT(handleFrame(EngineOutputPtr)), Qt::QueuedConnection);
    //connect(myEngine, SIGNAL(newFrame()), this, SLOT(handleFrame()), Qt::QueuedBlockingConnection);

    QToolBar* tb = addToolBar("Tools");
    QAction* action_quit = tb->addAction("Quit");
    QAction* action_about = tb->addAction("About");
    tb->addSeparator();
    QAction* action_start = tb->addAction("Start");
    QAction* action_stop = tb->addAction("Stop");
    tb->addSeparator();
    QAction* action_show_raw = tb->addAction("Raw");
    QAction* action_show_edges = tb->addAction("Edges");
    QAction* action_show_traces = tb->addAction("Traces");
    QAction* action_show_circles = tb->addAction("Circles");
    tb->addSeparator();
    QAction* action_home = tb->addAction("Home");

    QActionGroup* group_image = new QActionGroup(this);
    group_image->addAction(action_show_raw);
    group_image->addAction(action_show_edges);
    group_image->addAction(action_show_traces);
    group_image->addAction(action_show_circles);
    action_show_raw->setCheckable(true);
    action_show_edges->setCheckable(true);
    action_show_traces->setCheckable(true);
    action_show_circles->setCheckable(true);
    action_show_raw->setChecked(true);

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
    connect(action_home, SIGNAL(triggered()), myViewer, SLOT(home()));
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
    EngineConfigPtr config = ConfigDialog::askConfig(myCalibrationModel, this);

    if(config)
    {
        myEngine->setConfig(config);
        myActionStart->setEnabled(false);
        myActionStop->setEnabled(false);
        myEngine->start();
    }
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

void MainWindow::handleFrame(EngineOutputPtr frame)
{
    std::cout << "HANDLE " << frame->frame_id << std::endl;
}

