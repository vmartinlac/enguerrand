#include "VideoModule.h"
#include "CirclesDetection.h"
#include "EdgeDetectionCPU.h"
#include "EdgeDetectionGPU.h"
#include "EKFSLAM.h"
#include "VideoPort.h"
#include "EdgeCirclesPort.h"
#include "PoseLandmarksPort.h"
#include "SLAMPipeline.h"

SLAMPipeline::SLAMPipeline()
{
}

void SLAMPipeline::build(VideoSourcePtr video)
{
    VideoModule* video_module = new VideoModule();
    EdgeDetectionCPU* edge_module = new EdgeDetectionCPU();
    CirclesDetection* circles_module = new CirclesDetection();
    EKFSLAM* ekf_module = new EKFSLAM();

    video_module->setVideoSource(video);

    modules.resize(4);
    modules[0].reset(video_module);
    modules[1].reset(edge_module);
    modules[2].reset(circles_module);
    modules[3].reset(ekf_module);

    ports.resize(3);
    ports[0].reset(new GenericPipelinePortFactory<VideoPort>());
    ports[1].reset(new GenericPipelinePortFactory<EdgeCirclesPort>());
    ports[2].reset(new GenericPipelinePortFactory<PoseLandmarksPort>());

    connections.resize(8);

    connections[0] = 0;

    connections[1] = 0;
    connections[2] = 1;

    connections[3] = 0;
    connections[4] = 1;

    connections[5] = 0;
    connections[6] = 1;
    connections[7] = 2;

    lags.resize(4);
    lags[0] = 0;
    lags[1] = 0;
    lags[2] = 1;
    lags[3] = 2;

    thread_partition.resize(3);
    thread_partition[0] = 2;
    thread_partition[1] = 1;
    thread_partition[2] = 1;
}

