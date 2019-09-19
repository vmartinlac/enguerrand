#include <osg/ShapeDrawable>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/Geode>
#include "ViewerWidget.h"

osg::Vec3d ViewerWidget::vectorEigen2osg(const Eigen::Vector3d& x)
{
    return osg::Vec3d( x.x(), x.y(), x.z() );
}

osg::Quat ViewerWidget::quaternionEigen2osg(const Eigen::Quaterniond& q)
{
    return osg::Quat( q.x(), q.y(), q.z(), q.w() );
}


ViewerWidget::ViewerWidget(QWidget* parent) : ViewerWidgetBase(parent)
{
    myLandmarksGroup = new osg::Group();

    setMinimumSize(320, 200);

    // create node for landmark.

    {
        osg::ref_ptr<osg::Sphere> shape = new osg::Sphere(osg::Vec3d(0.0, 0.0, 0.0), 1.0);

        osg::ref_ptr<osg::ShapeDrawable> shape_drawable = new osg::ShapeDrawable(shape);
        shape_drawable->setColor(osg::Vec4d(0.0, 1.0, 0.0, 1.0)); // TODO: let the user configure this?

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(shape_drawable);

        myLandmarkNode = geode;

        myLandmarksGroup = new osg::Group();
    }

    // create node for camera.

    {
        // TODO: load node from resources.

        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
        geometry->setVertexArray(vertices);
        geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::DrawArrays::QUADS, 0, 4));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geometry);

        myCameraPAT = new osg::PositionAttitudeTransform();
        myCameraPAT->addChild(geode);
    }

    // create root node

    {
        osg::ref_ptr<osg::Group> root = new osg::Group();
        root->addChild(myLandmarksGroup);
        root->addChild(myCameraPAT);
        setSceneData(root);
    }
}

void ViewerWidget::handleFrame(EngineOutputPtr frame)
{
    myResult = frame;

    while(myResult->landmarks.size() >= myLandmarksPool.size() )
    {
        osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();

        transform->addChild(myLandmarkNode);

        myLandmarksPool.push_back(transform);
    }

    myLandmarksGroup->removeChildren(0, myLandmarksGroup->getNumChildren());

    for(size_t i=0; i<myLandmarksPool.size(); i++)
    {
        myLandmarksPool[i]->setPosition( vectorEigen2osg(myResult->landmarks[i].position) );
        myLandmarksGroup->addChild(myLandmarksPool[i]);
    }

    myCameraPAT->setPosition(vectorEigen2osg(myResult->current_frame.camera_to_world.translation()));
    myCameraPAT->setAttitude( quaternionEigen2osg(myResult->current_frame.camera_to_world.unit_quaternion()));
}

