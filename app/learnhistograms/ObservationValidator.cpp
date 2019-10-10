#include "ObservationValidator.h"

const double ObservationValidator::myTransformationExponent = (1.0/3.0);

const double ObservationValidator::mySVMNu = 0.5;

const double ObservationValidator::myRadiusRatio = 0.9;

const size_t ObservationValidator::myNumHistogramBins = 16;

ObservationValidator::ObservationValidator()
{
}

bool ObservationValidator::addSample(const cv::Mat3b& image, const cv::Vec3f& circle)
{
    Histogram hist;

    const bool ret = hist.build(myNumHistogramBins, image, circle, myRadiusRatio);

    if(ret)
    {
        myTrainingSet.push_back(std::move(hist));
    }

    return ret;
}

void ObservationValidator::clearSamples()
{
    myTrainingSet.clear();
}

bool ObservationValidator::load(const std::string& path)
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

void ObservationValidator::save(const std::string& path)
{
    mySVM->save(path);
}

bool ObservationValidator::train()
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

    return ret;
}

bool ObservationValidator::validate(const cv::Mat3b& image, const cv::Vec3f& circle)
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

size_t ObservationValidator::getNumSamples() const
{
    return myTrainingSet.size();
}

