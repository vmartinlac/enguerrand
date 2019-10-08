#include <iostream>
#include <QIcon>
#include <QKeySequence>
#include <QMessageBox>
#include <QApplication>
#include <QToolBar>
#include <QLabel>
#include <QSplitter>
#include "MainWindow.h"
#include "ViewerWidget.h"
#include "ConfigDialog.h"
#include "AboutDialog.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    myEngine = new Engine(this);
    myEngine->start();

    QToolBar* tb = addToolBar("Tools");
    tb->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

    QAction* action_quit = tb->addAction("Quit");
    tb->addSeparator();
    QAction* action_start = tb->addAction("Start");
    QAction* action_stop = tb->addAction("Stop");
    tb->addSeparator();
    QAction* action_show_raw = tb->addAction("Input");
    QAction* action_show_edges = tb->addAction("Edges");
    QAction* action_show_traces = tb->addAction("Traces");
    QAction* action_show_circles = tb->addAction("Detection");
    tb->addSeparator();
    QAction* action_home = tb->addAction("Home");
    tb->addSeparator();
    QAction* action_about = tb->addAction("About");

    action_quit->setIcon(QIcon::fromTheme("close-window"));
    action_about->setIcon(QIcon::fromTheme("about"));
    action_home->setIcon(QIcon::fromTheme("home"));
    action_start->setIcon(QIcon::fromTheme("play"));
    action_stop->setIcon(QIcon::fromTheme("stop"));
    action_show_raw->setIcon(QIcon::fromTheme("kiwi"));
    action_show_edges->setIcon(QIcon::fromTheme("melon"));
    action_show_traces->setIcon(QIcon::fromTheme("raspberry"));
    action_show_circles->setIcon(QIcon::fromTheme("olive"));

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

    action_start->setShortcut(QKeySequence("F5"));
    action_stop->setShortcut(QKeySequence("F6"));
    action_about->setShortcut(QKeySequence("F1"));
    action_quit->setShortcut(QKeySequence("Esc"));
    action_stop->setEnabled(false);

    myVideo = new VideoWidget();

    myViewer = new ViewerWidget();

    QSplitter* splitter = new QSplitter();
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(myVideo);
    splitter->addWidget(myViewer);

    setCentralWidget(splitter);
    setWindowTitle("Enguerrand");

    connect(myEngine, SIGNAL(engineStarted()), this, SLOT(engineStarted()), Qt::QueuedConnection);
    connect(myEngine, SIGNAL(engineStopped()), this, SLOT(engineStopped()), Qt::QueuedConnection);
    connect(myEngine, SIGNAL(newFrame(EngineOutputPtr)), myVideo, SLOT(handleFrame(EngineOutputPtr)), Qt::QueuedConnection);
    connect(myEngine, SIGNAL(newFrame(EngineOutputPtr)), myViewer, SLOT(handleFrame(EngineOutputPtr)), Qt::QueuedConnection);

    connect(action_about, SIGNAL(triggered()), this, SLOT(about()));
    connect(action_start, SIGNAL(triggered()), this, SLOT(startEngine()));
    connect(action_stop, SIGNAL(triggered()), this, SLOT(stopEngine()));
    connect(action_quit, SIGNAL(triggered()), this, SLOT(close()));
    connect(action_home, SIGNAL(triggered()), myViewer, SLOT(home()));
    connect(action_home, SIGNAL(triggered()), myVideo, SLOT(home()));
    connect(action_show_raw, SIGNAL(triggered()), myVideo, SLOT(displayInputImage()));
    connect(action_show_edges, SIGNAL(triggered()), myVideo, SLOT(displayEdgesImage()));
    connect(action_show_traces, SIGNAL(triggered()), myVideo, SLOT(displayTraceImage()));
    connect(action_show_circles, SIGNAL(triggered()), myVideo, SLOT(displayDetectionImage()));
}

void MainWindow::closeEvent(QCloseEvent* ev)
{
    myEngine->disconnect();
    QMetaObject::invokeMethod(myEngine, "stopEngine", Qt::QueuedConnection); // Engine is robust to superfluous calls to stopEngine().
    myEngine->quit();
    myEngine->wait();
    QMainWindow::closeEvent(ev);
}

MainWindow::~MainWindow()
{
}

void MainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void MainWindow::startEngine()
{
    EngineConfigPtr config = ConfigDialog::askConfig(this);

    if(config)
    {
        myActionStart->setEnabled(false);
        myActionStop->setEnabled(false);
        QMetaObject::invokeMethod(myEngine, "startEngine", Qt::QueuedConnection, Q_ARG(EngineConfigPtr,config));
    }
}

void MainWindow::stopEngine()
{
    myActionStart->setEnabled(false);
    myActionStop->setEnabled(false);
    QMetaObject::invokeMethod(myEngine, "stopEngine", Qt::QueuedConnection);
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

