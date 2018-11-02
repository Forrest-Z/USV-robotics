#include "dialogfixedsetpoint.h"
#include "ui_dialogfixedsetpoint.h"

Dialogfixedsetpoint::Dialogfixedsetpoint(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialogfixedsetpoint)
{
    ui->setupUi(this);
}

Dialogfixedsetpoint::~Dialogfixedsetpoint()
{
    delete ui;
}
