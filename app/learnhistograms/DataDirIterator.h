
#pragma once

#include <map>
#include <opencv2/core.hpp>
#include <QDir>

class DataDirIterator
{
public:

    DataDirIterator();

    void reset(const QDir& dir);

    bool next(cv::Vec3f& circle, cv::Mat3b& image);

protected:

    QDir myDir;
    std::map<std::string,cv::Vec3f> myListing;
};

