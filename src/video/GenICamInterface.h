
#pragma once

#include "GenICamVideoSource.h"

class GenICamInterface
{
public:

    static void initialize();

    static GenICamInterface* instance();

    GenICamInterface();

    void discover();

    GenICamVideoSourcePtr createVideoSourceMono(const std::string& device);

    GenICamVideoSourcePtr createVideoSourceStereo(const std::string& device0, const std::string& device1);

    GenICamVideoSourcePtr createVideoSourceMulti(const std::vector<std::string>& devices);

protected:

    static std::unique_ptr<GenICamInterface> mInstance;
};

