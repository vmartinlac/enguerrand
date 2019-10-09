
#pragma once

#include <map>
#include <opencv2/core.hpp>
#include <QDir>

/**
Iterator interface.
*/
class DataDirIterator
{
public:

    DataDirIterator();

    virtual void reset(const QDir& dir) = 0;

    virtual bool next(cv::Vec3f& circle, cv::Mat3b& image) = 0;
};

/**
Parse a directory containing an index file and images.
*/
class DataDirIterator1 : public DataDirIterator
{
public:

    DataDirIterator1();

    void reset(const QDir& dir) override;

    bool next(cv::Vec3f& circle, cv::Mat3b& image) override;

protected:

    QDir myDir;
    std::map<std::string,cv::Vec3f> myListing;
};

/**
Parse a directory containing other directories of the kind which can be parsed by DataDirIterator1.
*/
class DataDirIterator2 : public DataDirIterator
{
public:

    DataDirIterator2();

    void reset(const QDir& dir) override;

    bool next(cv::Vec3f& circle, cv::Mat3b& image) override;

protected:

    bool myReady;
    QDir myDir;
    QStringList mySubDirectories;
    DataDirIterator1 myIterator1;
};
