#include "ViewerWidget.h"

ViewerWidget::ViewerWidget(QWidget* parent) : ViewerWidgetBase(parent)
{
    setMinimumSize(320, 200);
}

void ViewerWidget::handleFrame(EngineOutputPtr frame)
{
    myResult = frame;
    std::cout << "ViewerWidget => handleFrame" << std::endl;
}

void ViewerWidget::initialize()
{
    ;
}

