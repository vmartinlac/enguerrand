#include <fstream>
#include <iostream>
#include <opencv2/ml.hpp>
#include <QDir>
#include "DataDirIterator.h"
#include "ObservationValidatorSimple.h"
#include "ObservationValidatorSVM.h"

void print_help_and_exit()
{
    std::cerr << "Invalid command line!" << std::endl;
    std::cerr << "Usage: learnobservations TYPE TRAINING_DATA OUTPUT_FILE" << std::endl;
    std::cerr << "TYPE is one of { Simple, SVM }." << std::endl;
    exit(1);
}

int main(int num_args, char** args)
{
    if(num_args != 4)
    {
        print_help_and_exit();
    }

    ObservationValidatorPtr ov;
    const std::string type(args[1]);
    if( type == "Simple" )
    {
        ov = std::make_shared<ObservationValidatorSimple>();
    }
    else if(type == "SVM")
    {
        ov = std::make_shared<ObservationValidatorSVM>();
    }
    else
    {
        print_help_and_exit();
    }

    std::cout << "Loading samples..." << std::endl;

    cv::Vec3f circle;
    cv::Mat3b image;

    DataDirIterator2 it;
    it.reset(QDir(args[2]));

    ov->prepareTraining();

    while(it.next(circle, image))
    {
        ov->addSample(image, circle);
    }

    std::cout << "Training model..." << std::endl;

    const bool ok = ov->train();

    if(ok)
    {
        std::cout << "Saving model..." << std::endl;
        ov->save("model.bin");
    }
    else
    {
        std::cerr << "Training failed!" << std::endl;
    }

    return 0;
}

