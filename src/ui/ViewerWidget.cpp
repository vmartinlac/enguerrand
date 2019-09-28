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

    myLandmarkRadius = 1.0;

    setMinimumSize(320, 200);

    // create node for landmark.

    {
        osg::ref_ptr<osg::Sphere> shape = new osg::Sphere(osg::Vec3d(0.0, 0.0, 0.0), myLandmarkRadius);

        osg::ref_ptr<osg::ShapeDrawable> shape_drawable = new osg::ShapeDrawable(shape);
        shape_drawable->setColor(osg::Vec4d(0.0, 1.0, 0.0, 1.0)); // TODO: let the user configure this?

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(shape_drawable);

        myLandmarkNode = geode;

        myLandmarksGroup = new osg::Group();
    }

    // create node for camera.

    {
        // TODO: use config?
        const double fx = 1.80993344e+03;
        const double fy = 1.80745383e+03;
        const double imw = 1920.0;
        const double imh = 1080.0;
        const double zz = myLandmarkRadius*5;

        // TODO: load node from resources?

        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        vertices->push_back( osg::Vec3d(0.0, 0.0, 0.0) );
        vertices->push_back( osg::Vec3d(-0.5*zz*imw/fx, -0.5*zz*imh/fy, zz) );
        vertices->push_back( osg::Vec3d(-0.5*zz*imw/fx, 0.5*zz*imh/fy, zz) );
        vertices->push_back( osg::Vec3d(0.5*zz*imw/fx, 0.5*zz*imh/fy, zz) );
        vertices->push_back( osg::Vec3d(0.5*zz*imw/fx, -0.5*zz*imh/fy, zz) );

        osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
        colors->push_back(osg::Vec3d(1.0, 0.0, 0.0));

        osg::ref_ptr<osg::DrawElementsUInt> primitive_set = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES);
        primitive_set->push_back(0);
        primitive_set->push_back(1);
        primitive_set->push_back(0);
        primitive_set->push_back(2);
        primitive_set->push_back(0);
        primitive_set->push_back(3);
        primitive_set->push_back(0);
        primitive_set->push_back(4);

        primitive_set->push_back(1);
        primitive_set->push_back(2);
        primitive_set->push_back(2);
        primitive_set->push_back(3);
        primitive_set->push_back(3);
        primitive_set->push_back(4);
        primitive_set->push_back(4);
        primitive_set->push_back(1);

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
        geometry->setVertexArray(vertices);
        geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
        geometry->addPrimitiveSet(primitive_set);

        //osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), myLandmarkRadius);
        //osg::ref_ptr<osg::ShapeDrawable> sphere_drawable = new osg::ShapeDrawable(sphere);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geometry);
        //geode->addDrawable(sphere_drawable);
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

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

    for(size_t i=0; i<myResult->landmarks.size(); i++)
    {
        myLandmarksPool[i]->setPosition( vectorEigen2osg(myResult->landmarks[i].position) );
        myLandmarksGroup->addChild(myLandmarksPool[i]);
    }

    myCameraPAT->setPosition(vectorEigen2osg(myResult->current_frame.camera_to_world.translation()));
    myCameraPAT->setAttitude( quaternionEigen2osg(myResult->current_frame.camera_to_world.unit_quaternion()));
}

