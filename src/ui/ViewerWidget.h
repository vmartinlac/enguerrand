
#pragma once

#include "EngineOutput.h"
#include "ViewerWidgetBase.h"

class ViewerWidget : public ViewerWidgetBase
{
    Q_OBJECT

public:

    ViewerWidget(QWidget* parent=nullptr);

public slots:

    void handleFrame(EngineOutputPtr frame);
};

