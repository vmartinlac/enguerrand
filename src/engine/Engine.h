
#pragma once

#include <QThread>
#include "EngineListener.h"
#include "EngineConfig.h"

class EngineContext;

class Engine : public QThread
{
public:

    Engine(QObject* parent=nullptr);

    void setListener(EngineListener* listener);

    void setConfig(EngineConfigPtr config);

private:

    void run() override;

private:

    EngineListener* myListener;
    EngineConfigPtr myConfig;
};

