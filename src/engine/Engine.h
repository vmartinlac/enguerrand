
#pragma once

#include <QThread>
#include "EngineConfig.h"
#include "EngineOutput.h"

class Engine : public QThread
{
    Q_OBJECT

public:

    Engine(QObject* parent=nullptr);

    void setConfig(EngineConfigPtr config);

signals:

    void newFrame(EngineOutputPtr frame);

private:

    void run() override;

private:

    EngineConfigPtr myConfig;
};

