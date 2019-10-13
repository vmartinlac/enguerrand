#include <iostream>
#include "EdgeDetection.h"
#include "EdgeDetectionCPU.h"

#ifdef WITH_CUDA
#include "EdgeDetectionGPU.h"
#endif

EdgeDetectionPtr EdgeDetection::createEdgeDetectionCPU()
{
    return std::make_shared<EdgeDetectionCPU>();
}

EdgeDetectionPtr EdgeDetection::createEdgeDetectionGPU()
{
#ifdef WITH_CUDA
    return std::make_shared<EdgeDetectionGPU>();
#else
    std::cerr << "Enguerrand was not compiled with CUDA support!" << std::endl;
    exit(1);
#endif
}

EdgeDetectionPtr EdgeDetection::createEdgeDetectionAny()
{
#ifdef WITH_CUDA
    return createEdgeDetectionGPU();
#else
    return createEdgeDetectionCPU();
#endif
}

