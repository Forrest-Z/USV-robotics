#include "dialogcooperation.h"
#include "ui_dialogcooperation.h"

DialogCooperation::DialogCooperation(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCooperation)
{
    ui->setupUi(this);
}

DialogCooperation::~DialogCooperation()
{
    delete ui;
}
