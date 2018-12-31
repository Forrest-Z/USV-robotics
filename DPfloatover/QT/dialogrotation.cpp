#include "dialogrotation.h"
#include "ui_dialogrotation.h"

DialogRotation::DialogRotation(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogRotation)
{
    ui->setupUi(this);
}

DialogRotation::~DialogRotation()
{
    delete ui;
}
