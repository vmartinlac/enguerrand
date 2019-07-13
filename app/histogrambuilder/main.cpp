#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <QDir>
#include "Histogram.h"

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }

    QDir directory(args[1]);

    QStringList files = directory.entryList(QDir::Files);

    HistogramBuilder builder;
    builder.init(64);

    for(const QString& f : files)
    {
        const std::string path = directory.absoluteFilePath(f).toStdString();
        cv::Mat3b image = cv::imread(path);

        if(image.data != nullptr)
        {
            builder.add(image);
        }
    }

    Histogram hist;
    builder.build(hist);
    hist.save("balls_histogram.bin");

    return 0;
}

