#include "ConfigDialog.h"

ConfigDialog::ConfigDialog(QWidget* parent) : QDialog(parent)
{
    myUI.setupUi(this);
}

EngineConfigPtr ConfigDialog::askConfig(QWidget* parent)
{
    EngineConfigPtr ret;

    ConfigDialog* dlg = new ConfigDialog(parent);

    if(dlg->exec() == QDialog::Accepted)
    {
        ret = std::make_shared<EngineConfig>();

        if( ret->loadFromFile("/home/victor/developpement/enguerrand/enguerrand/config/config_faber.json") == false ) ret.reset();
    }

    delete dlg;

    return ret;
}

