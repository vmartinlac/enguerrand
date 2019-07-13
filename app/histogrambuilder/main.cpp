#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <QDir>
#include "Histogram.h"

int main(int num_args, char** args)
{
    if(num_args != 4)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }


    const std::string thumbnails_directory(args[1]);
    const int num_bins = std::atoi(args[2]);
    const std::string output_path(args[3]);


    QDir directory(thumbnails_directory.c_str());

    QStringList files = directory.entryList(QDir::Files);

    BigHistogramBuilder builder;
    builder.init(num_bins);

    for(const QString& f : files)
    {
        const std::string path = directory.absoluteFilePath(f).toStdString();
        cv::Mat3b image = cv::imread(path);

        if(image.data != nullptr)
        {
            std::cout << "Loaded thumbnail " << path << std::endl;
            builder.add(image, 0.9);
        }
    }

    Histogram hist;
    if(builder.build(hist))
    {
        hist.save(output_path);
    }
    else
    {
        std::cerr << "Could not build histogram!" << std::endl;
        exit(1);
    }

    return 0;
}

