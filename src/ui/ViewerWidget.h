
#pragma once

#include <vector>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include "EngineOutput.h"
#include "ViewerWidgetBase.h"

class ViewerWidget : public ViewerWidgetBase
{
    Q_OBJECT

public:

    ViewerWidget(QWidget* parent=nullptr);

protected:

    static osg::Vec3d vectorEigen2osg(const Eigen::Vector3d& x);

    static osg::Quat quaternionEigen2osg(const Eigen::Quaterniond& q);

public slots:

    void handleFrame(EngineOutputPtr frame);

protected:

    osg::ref_ptr<osg::Node> myLandmarkNode;
    osg::ref_ptr<osg::PositionAttitudeTransform> myCameraPAT;
    osg::ref_ptr<osg::Group> myLandmarksGroup;
    std::vector< osg::ref_ptr<osg::PositionAttitudeTransform> > myLandmarksPool;
    EngineOutputPtr myResult;
};

