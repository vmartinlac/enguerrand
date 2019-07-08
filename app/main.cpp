#include <tbb/flow_graph.h>
#include <tbb/scalable_allocator.h>
#include <memory>
#include <iostream>
#include "FileVideoSource.h"
#include "EdgeCirclesData.h"

struct SLAMFrameCircle
{
    cv::Vec3f circle;
    size_t previous;
    bool has_previous;
};

class SLAMFrame
{
public:

    VideoFrame video_frame;
    cv::Mat2f normals;
    cv::Mat1b flags;
    std::vector<SLAMFrameCircle> circles;
};

typedef std::shared_ptr<SLAMFrame> SLAMFramePtr;

class VideoNodeBody
{
public:

    VideoNodeBody(VideoSourcePtr video)
    {
        mVideo = std::move(video);
    }

    VideoNodeBody(const VideoNodeBody& o)
    {
        mVideo = o.mVideo;
    }

    bool operator()(SLAMFramePtr& frame)
    {
        VideoFrame video_frame;
        mVideo->read(video_frame);

        if( video_frame.isValid() )
        {
            frame = std::allocate_shared<SLAMFrame,tbb::scalable_allocator<SLAMFrame>>(tbb::scalable_allocator<SLAMFrame>());
            frame->video_frame = std::move(video_frame);
            mVideo->trigger();
            return true;
        }
        else
        {
            return false;
        }
    }

protected:

    VideoSourcePtr mVideo;
};

class EdgeNodeBody
{
public:

    EdgeNodeBody()
    {
    }

    SLAMFramePtr operator()(SLAMFramePtr frame)
    {
        return frame;
    }
};

class CirclesNodeBody
{
public:

    CirclesNodeBody()
    {
    }

    SLAMFramePtr operator()(SLAMFramePtr frame)
    {
        return frame;
    }
};

int main(int num_args, char** args)
{
    tbb::flow::graph g;

    FileVideoSourcePtr video_source(new FileVideoSource());

    /*
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190608_120103.mp4");
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190608_120231.mp4");
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190615_150222.mp4");
    */
    video_source->setFileName("/home/victor/Développement/enguerrand/data/VID_20190615_180529.mp4");

    if( video_source->open() == false ) exit(1);

    video_source->trigger();

    tbb::flow::source_node<SLAMFramePtr> video_node(g, VideoNodeBody(video_source));
    tbb::flow::function_node<SLAMFramePtr,SLAMFramePtr> edge_node(g, 1, EdgeNodeBody());
    tbb::flow::function_node<SLAMFramePtr,SLAMFramePtr> circles_node(g, 1, CirclesNodeBody());

    make_edge(video_node, edge_node);
    make_edge(edge_node, circles_node);

    g.wait_for_all();

    video_source->close();

    return 0;
}

