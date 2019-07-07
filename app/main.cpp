#include <tbb/flow_graph.h>
#include <tbb/scalable_allocator.h>
#include <iostream>
#include "FileVideoSource.h"
#include "EdgeCirclesData.h"

class SLAMFrame
{
public:

    VideoFrame video_frame;
    cv::Mat3b input_image;
    EdgeCirclesData edge_circles_data;
};

typedef std::shared_ptr<SLAMFrame> SLAMFramePtr;

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
    video_source->setFileName("/home/victor/Développement/enguerrand/data/vid.mkv");

    if( video_source->open() == false ) exit(1);

    video_source->trigger();

    auto video_node_proc = [&video_source] (SLAMFramePtr& output) -> bool
    {
        video_source->read(output->video_frame);
        if( output->video_frame.isValid() )
        {
            video_source->trigger();
            output->input_image = output->video_frame.getView(0);
            return true;
        }
        else
        {
            return false;
        }
    };

    tbb::flow::source_node<SLAMFramePtr> video_node(g, video_node_proc);

    /*

    auto process_node_proc = [] (const VideoFrame& f) -> tbb::flow::continue_msg
    {
        return tbb::flow::continue_msg();
    };

    tbb::flow::function_node<VideoFrame,tbb::flow::continue_msg> process_node(g, 1, process_node_proc);

    tbb::flow::make_edge(video_node, process_node);

    g.wait_for_all();
    */

    return 0;
}

