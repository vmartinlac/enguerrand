#include "HistogramValidator.h"

HistogramValidator::HistogramValidator()
{
    myAlpha = 0.5;
}

bool HistogramValidator::load(const std::string& path)
{
    bool ret = false;
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::load(path);

    if( !svm.empty() )
    {
        mySVM = svm;
        myDimension = svm->getVarCount();
        ret = true;
    }

    return ret;
}

void HistogramValidator::save(const std::string& path)
{
    mySVM->save(path);
}

bool HistogramValidator::train(const std::vector<Histogram>& histograms)
{
    bool ret = false;

    if( histograms.empty() == false )
    {
        const size_t N = histograms.size();
        const size_t M = histograms.front().refVector().size();

        cv::Mat1f training_data(N,M);
        cv::Mat1i responses(N,1);

        for(size_t i=0; i<N; i++)
        {
            const Histogram& hist = histograms[i];
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
                training_data(i,j) = std::pow( float(v[j]) / float(count), myAlpha );
            }
        }

        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::ONE_CLASS);
        svm->setKernel(cv::ml::SVM::LINEAR); // TODO: check INTER
        svm->setNu(0.5);
        //svm->setTermCriteria(cv::TermCriteria());
        svm->train(training_data, cv::ml::ROW_SAMPLE, responses);

        if(svm->getVarCount() != M)
        {
            std::cerr << "Internal error!" << std::endl;
            exit(1);
        }

        mySVM = svm;
        myDimension = M;
        ret = true;
    }

    return ret;
}

bool HistogramValidator::validate(const Histogram& hist)
{
    const std::vector<uint32_t>& v = hist.refVector();
    const size_t count = hist.getCount();

    if(v.size() != myDimension)
    {
        std::cerr << "Internal error!" << std::endl;
        exit(1);
    }

    bool ret = false;

    if(count > 0)
    {
        cv::Mat1f item(1,myDimension);

        for(size_t j=0; j<myDimension; j++)
        {
            item(0,j) = std::pow( float(v[j]) / float(count), myAlpha );
        }

        ret = ( mySVM->predict(item) > 0.0f );;
    }

    return ret;
}

