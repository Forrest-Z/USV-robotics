#include "dialogsetpoints.h"
#include "ui_dialogsetpoints.h"

Dialogsetpoints::Dialogsetpoints(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogsetpoints) {
  ui->setupUi(this);
  initializesetpointDiag();
}

Dialogsetpoints::~Dialogsetpoints() { delete ui; }

void Dialogsetpoints::initializesetpointDiag() {
  QString valestring = QString::number(0);
  ui->LE_setxI0->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setyI0->setText(valestring);
  valestring = QString::number(0);
  ui->LE_setvelocityI->setText(valestring);

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
  double desired_velocity = ui->LE_setvelocityI->text().toDouble();
  double endx = ui->LE_setxI1->text().toDouble();
  double endy = ui->LE_setyI1->text().toDouble();
  double endtheta = ui->LE_setthetaI1->text().toDouble();
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->setupsetpointI->setEnabled(false);
  ui->resetsetpointI->setEnabled(true);
  ui->RB_straightlineI->setEnabled(false);
  ui->RB_crossI->setEnabled(false);
  ui->RB_fixedpointI->setEnabled(false);
  ui->RB_boxI->setEnabled(false);
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setvelocityI->setEnabled(false);
  ui->LE_setxI1->setEnabled(false);
  ui->LE_setyI1->setEnabled(false);
  ui->LE_setthetaI1->setEnabled(false);
}

void Dialogsetpoints::on_resetsetpointI_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->setupsetpointI->setEnabled(true);
  ui->resetsetpointI->setEnabled(false);
  ui->RB_straightlineI->setEnabled(true);
  ui->RB_crossI->setEnabled(true);
  ui->RB_fixedpointI->setEnabled(true);
  ui->RB_boxI->setEnabled(true);
  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setvelocityI->setEnabled(true);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}

void Dialogsetpoints::on_RB_fixedpointI_clicked() {
  globalvar::_threadloop.setsetpointmode_first(1);

  // ui operation
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setvelocityI->setEnabled(false);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}

void Dialogsetpoints::on_RB_straightlineI_clicked() {
  globalvar::_threadloop.setsetpointmode_first(2);

  // ui operation
  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setvelocityI->setEnabled(true);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}
