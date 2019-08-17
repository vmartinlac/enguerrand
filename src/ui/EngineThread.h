
#pragma once

#include <QThread>
#include "EngineConfig.h"

class EngineThread : public QThread
{
    Q_OBJECT

public:

    EngineThread(EngineConfigPtr config, QObject* parent=nullptr);

    void run() override;

protected:

    EngineConfigPtr myConfig;
};

