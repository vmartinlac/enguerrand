
#pragma once

#include <QThread>

class EngineThread : public QThread
{
public:

    EngineThread(QObject* parent=nullptr);
};

