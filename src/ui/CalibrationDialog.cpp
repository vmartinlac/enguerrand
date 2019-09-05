#include <QToolBar>
#include <QVBoxLayout>
#include <QTableView>
#include "CalibrationDialog.h"

CalibrationDialog::CalibrationDialog(CalibrationModel* model, QWidget* parent) : QDialog(parent)
{
    myModel = model;

    QToolBar* tb = new QToolBar();
    tb->addAction("New");
    tb->addAction("Modify");
    tb->addAction("Delete");

    QTableView* table = new QTableView();
    table->setModel(model);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(table);

    setLayout(lay);
    setWindowTitle("Edit calibrations");
}

