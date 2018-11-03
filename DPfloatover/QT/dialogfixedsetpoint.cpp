#include "dialogfixedsetpoint.h"
#include "ui_dialogfixedsetpoint.h"

Dialogfixedsetpoint::Dialogfixedsetpoint(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogfixedsetpoint) {
  ui->setupUi(this);
  initializeDialogfixedsetpoint();
}

Dialogfixedsetpoint::~Dialogfixedsetpoint() { delete ui; }

void Dialogfixedsetpoint::on_setupsetpointI_clicked() {
  double setxI = ui->LE_setxI->text().toDouble();
  double setyI = ui->LE_setyI->text().toDouble();
  double settheta = ui->LE_setthetaI->text().toDouble();
  settheta *= (M_PI / 180);

  globalvar::_threadloop.setFixedpoint_first(setxI, setyI, settheta);

  // ui operation
  ui->LE_setxI->setEnabled(false);
  ui->LE_setyI->setEnabled(false);
  ui->LE_setthetaI->setEnabled(false);
  ui->setupsetpointI->setEnabled(false);
  ui->resetsetpointI->setEnabled(true);
}

void Dialogfixedsetpoint::on_resetsetpointI_clicked() {
  // ui operation
  ui->LE_setxI->setEnabled(true);
  ui->LE_setyI->setEnabled(true);
  ui->LE_setthetaI->setEnabled(true);
  ui->setupsetpointI->setEnabled(true);
  ui->resetsetpointI->setEnabled(false);
}

void Dialogfixedsetpoint::on_setupsetpointII_clicked() {
  double setxI = ui->LE_setxII->text().toDouble();
  double setyI = ui->LE_setyII->text().toDouble();
  double settheta = ui->LE_setthetaII->text().toDouble();
  settheta *= (M_PI / 180);

  globalvar::_threadloop.setFixedpoint_second(setxI, setyI, settheta);

  // ui operation
  ui->LE_setxII->setEnabled(false);
  ui->LE_setyII->setEnabled(false);
  ui->LE_setthetaII->setEnabled(false);
  ui->setupsetpointII->setEnabled(false);
  ui->resetsetpointII->setEnabled(true);
}

void Dialogfixedsetpoint::on_resetsetpointII_clicked() {
  // ui operation
  ui->LE_setxII->setEnabled(true);
  ui->LE_setyII->setEnabled(true);
  ui->LE_setthetaII->setEnabled(true);
  ui->setupsetpointII->setEnabled(true);
  ui->resetsetpointII->setEnabled(false);
}

void Dialogfixedsetpoint::on_setupsetpointIII_clicked() {
  double setx = ui->LE_setxIII->text().toDouble();
  double sety = ui->LE_setyIII->text().toDouble();
  double settheta = ui->LE_setthetaIII->text().toDouble();
  settheta *= (M_PI / 180);

  globalvar::_threadloop.setFixedpoint_second(setx, sety, settheta);

  // ui operation
  ui->LE_setxIII->setEnabled(false);
  ui->LE_setyIII->setEnabled(false);
  ui->LE_setthetaIII->setEnabled(false);
  ui->setupsetpointIII->setEnabled(false);
  ui->resetsetpointIII->setEnabled(true);
}

void Dialogfixedsetpoint::on_resetsetpointIII_clicked() {
  // ui operation
  ui->LE_setxIII->setEnabled(true);
  ui->LE_setyIII->setEnabled(true);
  ui->LE_setthetaIII->setEnabled(true);
  ui->setupsetpointIII->setEnabled(true);
  ui->resetsetpointIII->setEnabled(false);
}

void Dialogfixedsetpoint::initializeDialogfixedsetpoint() {
  double setxI = 0;
  double setyI = 0;
  double setthetaI = 0;
  double setxII = 0;
  double setyII = 0;
  double setthetaII = 0;
  double setxIII = 0;
  double setyIII = 0;
  double setthetaIII = 0;
  globalvar::_threadloop.getfixedpointdata_first(setxI, setyI, setthetaI);
  globalvar::_threadloop.getfixedpointdata_second(setxII, setyII, setthetaII);
  globalvar::_threadloop.getfixedpointdata_third(setxIII, setyIII, setthetaIII);

  QString valestring = QString::number(setxI);
  ui->LE_setxI->setText(valestring);
  valestring = QString::number(setyI);
  ui->LE_setyI->setText(valestring);
  valestring = QString::number(setthetaI);
  ui->LE_setthetaI->setText(valestring);

  valestring = QString::number(setxII);
  ui->LE_setxII->setText(valestring);
  valestring = QString::number(setyII);
  ui->LE_setyII->setText(valestring);
  valestring = QString::number(setthetaII);
  ui->LE_setthetaII->setText(valestring);

  valestring = QString::number(setxIII);
  ui->LE_setxIII->setText(valestring);
  valestring = QString::number(setyIII);
  ui->LE_setyIII->setText(valestring);
  valestring = QString::number(setthetaIII);
  ui->LE_setthetaIII->setText(valestring);
}