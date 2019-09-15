
#pragma once

#include <QThread>
#include "EngineConfig.h"
#include "EngineOutput.h"
#include "EngineGraph.h"

class Engine : public QThread
{
    Q_OBJECT

public:

    Engine(QObject* parent=nullptr);

public slots:

    void startEngine(EngineConfigPtr config);
    void stopEngine();

signals:

    void engineStarted();
    void engineStopped();
    void newFrame(EngineOutputPtr frame);

private:

    EngineConfigPtr myConfig;
    bool myIsEngineRunning;
    std::atomic<bool> myExitRequested;
    std::unique_ptr<EngineGraph::AsyncVideoCallback> myAsyncVideoCallback;
    std::unique_ptr<tbb::flow::graph> myGraph;
    std::unique_ptr<EngineGraph::VideoNode> myVideoNode;
    std::unique_ptr<EngineGraph::VideoLimiterNode> myVideoLimiterNode;
    std::unique_ptr<EngineGraph::EdgeNode> myEdgeNode;
    std::unique_ptr<EngineGraph::VideoEdgeJoinNode> myVideoEdgeJoinNode;
    std::unique_ptr<EngineGraph::CircleNode> myCirclesNode;
    std::unique_ptr<EngineGraph::OdometryNode> myOdometryNode;
    std::unique_ptr<EngineGraph::TracesNode> myTracesNode;
    std::unique_ptr<EngineGraph::VideoEdgeCirclesOdometryTracesJoinNode> myVideoEdgeCirclesOdometryTracesJoinNode;
    std::unique_ptr<EngineGraph::TerminalNode> myTerminalNode;
};

