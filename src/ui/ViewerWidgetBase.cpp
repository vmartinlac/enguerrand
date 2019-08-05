#include <iostream>
#include <osgGA/TrackballManipulator>
#include <QEvent>
#include <QTimerEvent>
#include <QMouseEvent>
#include "ViewerWidgetBase.h"

ViewerWidgetBase::ViewerWidgetBase(QWidget* parent) :
    QOpenGLWidget(parent)
{
    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;

    mViewer = new osgViewer::Viewer;
    mViewer->setCameraManipulator(manipulator);
    mViewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    mViewer->setRunFrameScheme(osgViewer::Viewer::ON_DEMAND);

    mWindow = mViewer->setUpViewerAsEmbeddedInWindow(0, 0, width(), height());

    //setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(100, 100);
    mUpdateTimer = startTimer(30);

    mViewer->home();
}

void ViewerWidgetBase::setSceneData(osg::ref_ptr<osg::Node> node)
{
    mViewer->setSceneData(node);
}

void ViewerWidgetBase::timerEvent(QTimerEvent* ev)
{
    if(ev->timerId() == mUpdateTimer)
    {
        update();
    }
}

void ViewerWidgetBase::paintGL()
{
    mViewer->frame();
}

void ViewerWidgetBase::mouseMoveEvent(QMouseEvent* event)
{
    mWindow->getEventQueue()->mouseMotion(event->x(), event->y());
}

void ViewerWidgetBase::initializeGL()
{
   ;
}

void ViewerWidgetBase::mousePressEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    mWindow->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
}

void ViewerWidgetBase::mouseReleaseEvent(QMouseEvent* event)
{
    unsigned int button = 0;

    switch (event->button())
    {
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }

    mWindow->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void ViewerWidgetBase::wheelEvent(QWheelEvent* event)
{
    const int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?  osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
    mWindow->getEventQueue()->mouseScroll(motion);
}

bool ViewerWidgetBase::event(QEvent* event)
{
    bool handled = QOpenGLWidget::event(event);

    switch( event->type() )
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
        update();
        break;
    /*
    case QEvent::Timer:
        update();
        break;
    */
    default:
        break;
    };

    return handled;
}

void ViewerWidgetBase::resizeGL(int width, int height)
{
    //mWindow->getEventQueue()->windowResize( this->x(), this->y(), width, height );
    mWindow->resized( this->x(), this->y(), width, height );

   /*
   std::vector<osg::Camera*> cameras;
   mViewer->getCameras( cameras );

   assert( cameras.size() == 1 );

   cameras.front()->setViewport( 0, 0, this->width(), this->height() );
   */
}

void ViewerWidgetBase::home()
{
    mViewer->home();
}

