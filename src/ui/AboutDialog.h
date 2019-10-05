#pragma once

#include <QDialog>
#include "ui_AboutDialog.h"

class AboutDialog : public QDialog
{
public:

    AboutDialog(QWidget* parent=nullptr);

protected:

    Ui::AboutDialog myUI;
};

