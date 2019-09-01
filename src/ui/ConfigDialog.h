
#pragma once

#include <QDialog>
#include "EngineConfig.h"
#include "ui_ConfigDialog.h"

class ConfigDialog : public QDialog
{

Q_OBJECT

public:

    ConfigDialog(QWidget* parent=nullptr);

    static EngineConfigPtr askConfig(QWidget* parent=nullptr);

protected:

    Ui::ConfigDialog myUI;
};

