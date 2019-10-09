#include <fstream>
#include <iostream>
#include <opencv2/ml.hpp>
#include <QDir>
#include "Histogram.h"
#include "DataDirIterator.h"
#include "HistogramValidator.h"

#define HISTOGRAM_NUM_BINS 16

#define GAMMA (0.3333)

void loadHistograms(const QString& path, std::vector<Histogram>& histograms)
{
    cv::Vec3f circle;
    cv::Mat3b image;

    DataDirIterator2 it;
    it.reset(QDir(path));

    while(it.next(circle, image))
    {
        Histogram hist;
        if( hist.build(HISTOGRAM_NUM_BINS, image, circle, 0.9) )
        {
            histograms.push_back(std::move(hist));
        }
    }
}

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }

    std::cout << "Loading histograms..." << std::endl;

    std::vector<Histogram> histograms;
    loadHistograms(args[1], histograms);

    std::cout << "Number of histograms: " << histograms.size() << std::endl;

    if(histograms.empty())
    {
        std::cerr << "No available histogram!" << std::endl;
        exit(1);
    }

    HistogramValidatorPtr validator = std::make_shared<HistogramValidator>();

    std::cout << "Training model..." << std::endl;
    validator->train(histograms);

    std::cout << "Saving model..." << std::endl;
    validator->save("model.bin");

    return 0;
}

