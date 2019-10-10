#include "ObservationValidatorSVM.h"

const double ObservationValidatorSVM::myTransformationExponent = (1.0/3.0);
const double ObservationValidatorSVM::mySVMNu = 0.5;
const double ObservationValidatorSVM::myRadiusRatio = 0.95;
const size_t ObservationValidatorSVM::myNumHistogramBins = 16;

ObservationValidatorSVM::ObservationValidatorSVM()
{
}

void ObservationValidatorSVM::addSample(const cv::Mat3b& image, const cv::Vec3f& circle)
{
    Histogram hist;

    const bool ret = hist.build(myNumHistogramBins, image, circle, myRadiusRatio);

    if(ret)
    {
        myTrainingSet.push_back(std::move(hist));
    }
}

bool ObservationValidatorSVM::load(const std::string& path)
{
    bool ret = false;
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load(path);

    if( !svm.empty() )
    {
        mySVM = svm;
        ret = true;
    }

    return ret;
}

bool ObservationValidatorSVM::save(const std::string& path)
{
    mySVM->save(path);
    return true;
}

void ObservationValidatorSVM::prepareTraining()
{
    myTrainingSet.clear();
}

bool ObservationValidatorSVM::train()
{
    bool ret = false;

    if( myTrainingSet.empty() == false )
    {
        const size_t N = myTrainingSet.size();
        const size_t M = myTrainingSet.front().refVector().size();

        cv::Mat1f training_data(N,M);
        cv::Mat1i responses(N,1);

        for(size_t i=0; i<N; i++)
        {
            const Histogram& hist = myTrainingSet[i];
            const std::vector<uint32_t>& v = hist.refVector();
            const size_t count = hist.getCount();

            if(v.size() != M || count == 0)
            {
                std::cerr << "Internal error!" << std::endl;
                exit(1);
            }

            responses(i,0) = 1;

            for(size_t j=0; j<M; j++)
            {
                training_data(i,j) = std::pow( float(v[j]) / float(count), myTransformationExponent );
            }
        }

        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::ONE_CLASS);
        svm->setKernel(cv::ml::SVM::LINEAR); // TODO: check INTER
        svm->setNu(mySVMNu);
        //svm->setTermCriteria(cv::TermCriteria());
        svm->train(training_data, cv::ml::ROW_SAMPLE, responses);

        if(svm->getVarCount() != M)
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        mySVM = svm;
        ret = true;
    }

    myTrainingSet.clear();

    return ret;
}

bool ObservationValidatorSVM::validate(const cv::Mat3b& image, const cv::Vec3f& circle)
{
    Histogram hist;
    bool ret = false;
    
    if( hist.build(myNumHistogramBins, image, circle, myRadiusRatio) )
    {
        const std::vector<uint32_t>& v = hist.refVector();
        const size_t count = hist.getCount();

        if( count == 0 || v.size() != mySVM->getVarCount() )
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        cv::Mat1f item(1,v.size());

        for(size_t j=0; j<v.size(); j++)
        {
            item(0,j) = std::pow( float(v[j]) / float(count), myTransformationExponent );
        }

        ret = ( mySVM->predict(item) > 0.0f );;
    }

    return ret;
}

///////////

ObservationValidatorSVM::Histogram::Histogram()
{
    myBins = 0;
}

bool ObservationValidatorSVM::Histogram::build(size_t bins, const cv::Mat3b& image, const cv::Vec3f& circle, double gamma)
{
    const size_t N = bins*bins*bins;

    myHistogram.assign(N, 0);

    myBins = bins;

    myCount = 0;

    const cv::Rect ROI =
        cv::Rect( cv::Point(0,0), image.size() ) &
        cv::Rect( circle[0]-circle[2]-1, circle[1]-circle[2]-1, 2*circle[2]+1, 2*circle[2]+1 );

    for(int di=0; di<ROI.height; di++)
    {
        for(int dj=0; dj<ROI.width; dj++)
        {
            const int j = ROI.x + dj;
            const int i = ROI.y + di;

            const double radius = std::hypot( j+0.5-circle[0], i+0.5-circle[1] );

            if(radius < gamma * circle[2])
            {
                cv::Vec3i tmp = image(i,j);
                tmp *= (int) bins;
                tmp /= 256;

                const size_t index = myBins*myBins*tmp[0] + myBins*tmp[1] + tmp[2];

                myHistogram[index]++;
                myCount++;
            }
        }
    }

    return (myCount > 0);
}

size_t ObservationValidatorSVM::Histogram::getBins() const
{
    return myBins;
}

size_t ObservationValidatorSVM::Histogram::getCount() const
{
    return myCount;
}

const std::vector<uint32_t>& ObservationValidatorSVM::Histogram::refVector() const
{
    return myHistogram;
}

std::vector<uint32_t>& ObservationValidatorSVM::Histogram::refVector()
{
    return myHistogram;
}

