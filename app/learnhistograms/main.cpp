#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <QDir>
#include "Histogram.h"
#include "DataDirIterator.h"
#include "HistogramValidator.h"

#include <svm.h>

#define HISTOGRAM_NUM_BINS 16

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }

    HistogramValidatorPtr validator = HistogramValidator::createHistogramValidatorSVM();

    QDir dir(args[1]);

    QStringList content = dir.entryList(QDir::Dirs);
    for(QString& item : content)
    {
        QDir data_dir = dir;
        if(data_dir.cd(item))
        {
            DataDirIterator it;
            it.reset(data_dir);

            cv::Vec3f circle;
            cv::Mat3b image;

            while(it.next(circle, image))
            {
                Histogram hist;
                if( hist.build(HISTOGRAM_NUM_BINS, image, circle, 0.9) )
                {
                    validator->addToTrainingSet(hist);
                }
            }
        }
    }

    validator->train();

    return 0;
}

