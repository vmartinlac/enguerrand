#include <fstream>
#include <iostream>
#include <opencv2/ml.hpp>
#include <QDir>
#include "Histogram.h"
#include "DataDirIterator.h"
#include "ObservationValidator.h"

void loadHistograms(const QString& path, std::vector<Histogram>& histograms)
{
}

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }

    ObservationValidator ov;

    std::cout << "Loading samples..." << std::endl;

    cv::Vec3f circle;
    cv::Mat3b image;

    DataDirIterator2 it;
    it.reset(QDir(args[1]));

    while(it.next(circle, image))
    {
        ov.addSample(image, circle);
    }

    std::cout << "Number of histograms: " << ov.getNumSamples() << std::endl;

    std::cout << "Training model..." << std::endl;
    const bool ok = ov.train();
    ov.clearSamples();

    if(ok)
    {
        std::cout << "Saving model..." << std::endl;
        ov.save("model.bin");
    }
    else
    {
        std::cerr << "Training failed!" << std::endl;
    }

    return 0;
}

