#include "dialogsetpoints.h"
#include "ui_dialogsetpoints.h"

Dialogsetpoints::Dialogsetpoints(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogsetpoints) {
  ui->setupUi(this);
}

Dialogsetpoints::~Dialogsetpoints() { delete ui; }

void Dialogsetpoints::initializesetpointDiag() {
  QString valestring = QString::number(0);
  ui->LE_setxI0->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setyI0->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setthetaI0->setText(valestring);

  valestring = QString::number(0);
  ui->LE_setxI1->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setyI1->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setthetaI1->setText(valestring);
}
void Dialogsetpoints::on_setupsetpointI_clicked() {
  double startx = ui->LE_setxI0->text().toDouble();
  double starty = ui->LE_setyI0->text().toDouble();
  double starttheta = ui->LE_setthetaI0->text().toDouble();
  double endx = ui->LE_setxI1->text().toDouble();
  double endy = ui->LE_setyI1->text().toDouble();
  double endtheta = ui->LE_setthetaI1->text().toDouble();
}

void Dialogsetpoints::on_resetsetpointI_clicked() {
  // ui operation
  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setthetaI0->setEnabled(true);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}

void Dialogsetpoints::on_RB_fixedpointI_clicked() {
  // ui operation
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setthetaI0->setEnabled(false);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}

void Dialogsetpoints::on_RB_straightlineI_clicked() {
  // ui operation
  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setthetaI0->setEnabled(true);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}
