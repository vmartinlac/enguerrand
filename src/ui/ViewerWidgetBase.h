#pragma once

#include <memory>
#include <QOpenGLWidget>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/Array>
#include <osgViewer/GraphicsWindow>

class QMouseEvent;

class ViewerWidgetBase : public QOpenGLWidget
{
    Q_OBJECT

public:

    ViewerWidgetBase(QWidget* parent=nullptr);
    
    void setSceneData(osg::ref_ptr<osg::Node> node);

protected:

    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    bool event(QEvent* event) override;
    void timerEvent(QTimerEvent* event) override;
    void initializeGL() override;

public slots:

    void home();

private:

    osg::ref_ptr<osgViewer::Viewer> mViewer;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> mWindow;
    int mUpdateTimer;
};

