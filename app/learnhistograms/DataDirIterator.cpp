#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include "DataDirIterator.h"

#define LISTING_FILE_NAME "listing.txt"

DataDirIterator::DataDirIterator()
{
}

void DataDirIterator::reset(const QDir& dir)
{
    myListing.clear();
    myDir = dir;

    std::ifstream file(dir.absoluteFilePath(LISTING_FILE_NAME).toStdString());

    if(file.is_open())
    {
        while(file.good())
        {
            std::string id;
            cv::Vec3f circle;

            file >> id;
            file >> circle[0];
            file >> circle[1];
            file >> circle[2];

            if(file.good())
            {
                myListing[id] = circle;
            }
        }
    }
}

bool DataDirIterator::next(cv::Vec3f& circle, cv::Mat3b& image)
{
    bool ret = true;
    bool go_on = true;

    while(go_on)
    {
        if(myListing.empty())
        {
            ret = false;
            go_on = false;
        }
        else
        {
            auto it = myListing.begin();
            std::pair<std::string,cv::Vec3f> item = *it;
            myListing.erase(it);

            const std::string fname = item.first + "_image.png";
            const std::string full_path = myDir.absoluteFilePath(fname.c_str()).toStdString();
            image = cv::imread(full_path);
            if(image.data)
            {
                circle = item.second;
                ret = true;
                go_on = false;
            }
        }
    }

    return ret;
}

