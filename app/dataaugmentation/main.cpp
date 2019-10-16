#include <QDir>
#include <QRegExp>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include "VariantGenerator.h"

class IdentityVariantGenerator : public VariantGenerator
{
protected:

    bool generateVariant(const cv::Mat3b& original, cv::Mat3b& variant) override
    {
        variant = original.clone();
        return true;
    }
};

class App
{
public:

    int run(int& num_args, char**& args)
    {
        buildVariantGenerators();

        QDir dir(args[1]);
        QStringList content = dir.entryList(QDir::Dirs);

        size_t count = 0;

        QRegExp regdir("^[0-9]+$");
        for( const QString& item : content )
        {
            if(regdir.exactMatch(item))
            {
                count += processDirectory(dir, item);
            }
        }

        std::cout << "Number of images generated: " << count << std::endl;

        return 0;
    }

protected:

    void buildVariantGenerators()
    {
        myGenerators.clear();

        myGenerators.emplace_back(new IdentityVariantGenerator());
    }

    size_t processDirectory(const QDir& dir, const QString& path)
    {
        std::cout << "Processing " << path.toStdString() << std::endl;

        QDir indir;
        QDir outdir;
        const QString outname = path + "_generated";
        std::ifstream inlisting;
        std::ofstream outlisting;
        cv::Mat3b inimage;
        cv::Mat3b outimage;

        QRegExp regexp("^[0-9]{6,6}_image.png$");

        bool ok = true;
        size_t ret = 0;

        if(ok)
        {
            indir = dir;
            ok = indir.cd(path);
        }

        if(ok)
        {
            if(dir.exists(outname))
            {
                outdir = dir;
                ok = outdir.cd(outname);
                if(ok)
                {
                    outdir.removeRecursively();
                }
            }
        }

        if(ok)
        {
            dir.mkdir(outname);

            QDir outdir = dir;
            ok = outdir.cd(outname);
        }

        if(ok)
        {
            inlisting = std::ifstream( indir.absoluteFilePath("listing.txt").toStdString().c_str() );
            outlisting = std::ofstream( outdir.absoluteFilePath("listing.txt").toStdString().c_str() );
            ok = inlisting.is_open() && outlisting.is_open();
        }

        if(ok)
        {
            bool go_on = true;

            while(ok && go_on)
            {
                std::string inid;
                cv::Vec3d circle;

                inlisting >> inid;
                inlisting >> circle[0];
                inlisting >> circle[1];
                inlisting >> circle[2];

                go_on = !inlisting.fail();

                if(go_on)
                {
                    const QString inimagepath = indir.absoluteFilePath( QString(inid.c_str()) + QString("_image.png") );

                    inimage = cv::imread( inimagepath.toStdString().c_str() );

                    if(inimage.data)
                    {
                        for(auto& gen : myGenerators)
                        {
                            if( (*gen)(inimage, outimage) )
                            {
                                const QString outid = QString("%1").arg(ulong(ret), 6, 10, QChar('0'));

                                const QString outimagepath = outdir.absoluteFilePath( QString(outid) + QString("_image.png") );

                                if( cv::imwrite(outimagepath.toStdString(), outimage) )
                                {
                                    outlisting 
                                        << outid.toStdString() << " "
                                        << circle[0] << " "
                                        << circle[1] << " "
                                        << circle[2] << std::endl;

                                    ret++;
                                }
                            }
                        }
                    }
                }
            }
        }

        inlisting.close();
        outlisting.close();

        return ret;
    }

protected:

    using VariantGeneratorVector = std::vector< std::unique_ptr<VariantGenerator> >;

protected:

    VariantGeneratorVector myGenerators;
};

int main(int num_args, char** args)
{
    return App().run(num_args, args);
}

