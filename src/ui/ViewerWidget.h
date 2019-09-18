
#pragma once

#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include "EngineOutput.h"
#include "ViewerWidgetBase.h"

class ViewerWidget : public ViewerWidgetBase
{
    Q_OBJECT

public:

    ViewerWidget(QWidget* parent=nullptr);

    void initialize();

public slots:

    void handleFrame(EngineOutputPtr frame);

protected:

    struct Landmark
    {
        osg::ref_ptr<osg::PositionAttitudeTransform> node;
    };

protected:

    osg::ref_ptr<osg::ShapeDrawable> myLandmarkDrawable;
    osg::ref_ptr<osg::Geometry> myCameraNode;
    osg::ref_ptr<osg::Group> myRootNode;
    EngineOutputPtr myResult;
};

