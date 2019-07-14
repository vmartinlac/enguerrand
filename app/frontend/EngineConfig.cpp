#include <QJsonObject>
#include <QJsonDocument>
#include <QFile>
#include "EKFOdometry.h"
#include "FileVideoSource.h"
#include "EngineConfig.h"

bool EngineConfig::loadFromFile(const std::string& path)
{
    QFile file(path.c_str());
    QJsonDocument doc;
    QJsonObject root;

    bool ret = true;
    const char* err = "";

    if(ret)
    {
        ret = file.open(QIODevice::ReadOnly);
        err = "Could not open config file!";
    }

    if(ret)
    {
        doc = QJsonDocument::fromJson(file.readAll());
        ret = doc.isObject();
    }

    if(file.isOpen())
    {
        file.close();
    }

    if(ret)
    {
        root = doc.object();
    }

    // set video_input.

    if(ret)
    {
        FileVideoSourcePtr file_video_source(new FileVideoSource());
        file_video_source->setFileName( root["video_input"].toObject()["path"].toString().toStdString() );
        video_input = file_video_source;
        err = "Could not set video input!";
    }

    // set balls_histogram.

    if(ret)
    {
        balls_histogram.reset(new Histogram());
        ret = balls_histogram->load( root["balls_histogram"].toString().toStdString() );
        err = "Could not load histogram!";
    }

    // set odometry_code.

    if(ret)
    {
        odometry_code.reset(new EKFOdometry());
        err = "Could not set odometry code!";
    }

    // if fail, reset.

    if(ret == false)
    {
        std::cerr << err << std::endl;
        clear();
    }

    return ret;
}

void EngineConfig::clear()
{
    video_input.reset();
    odometry_code.reset();
}

