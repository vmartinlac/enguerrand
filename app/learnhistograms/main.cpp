#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <QDir>
#include "Histogram.h"
#include "DataDirIterator.h"

#include <svm.h>

#define HISTOGRAM_NUM_BINS 16

struct TrainingData
{
    std::vector< std::vector<svm_node> > nodes;
    std::vector<double> probabilities;
};

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Invalid command line!" << std::endl;
        exit(1);
    }

    TrainingData tdata;

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
                    std::vector<svm_node> nodes;

                    auto& vector = hist.refVector();
                    for(size_t i=0; i<vector.size(); i++)
                    {
                        if( vector[i] > 0 )
                        {
                            nodes.emplace_back();
                            nodes.back().index = i;
                            nodes.back().value = double(vector[i]) / double(hist.getCount());
                        }
                    }

                    nodes.emplace_back();
                    nodes.back().index = -1;
                    nodes.back().value = 0.0;

                    tdata.nodes.push_back( std::move(nodes) );
                }
            }
        }
    }

    return 0;
}

