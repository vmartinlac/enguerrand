#include "EngineThread.h"

EngineThread::EngineThread(EngineConfigPtr config, QObject* parent) : QThread(parent)
{
    myConfig = config;
}

void EngineThread::run()
{
    QThread::run();
}

