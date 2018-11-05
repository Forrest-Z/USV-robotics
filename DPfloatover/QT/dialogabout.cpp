#include "dialogabout.h"
#include "ui_dialogabout.h"

Dialogabout::Dialogabout(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialogabout)
{
    ui->setupUi(this);
}

Dialogabout::~Dialogabout()
{
    delete ui;
}
