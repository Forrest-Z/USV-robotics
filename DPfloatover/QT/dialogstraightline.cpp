#include "dialogstraightline.h"
#include "ui_dialogstraightline.h"

Dialogstraightline::Dialogstraightline(QWidget *parent)
    : QDialog(parent), ui(new Ui::Dialogstraightline) {
  ui->setupUi(this);
  initializeStraightlineDiag();
}

Dialogstraightline::~Dialogstraightline() { delete ui; }

void Dialogstraightline::on_setupsetpointII_clicked() {
  double startx = ui->LE_setxII0->text().toDouble();
  double starty = ui->LE_setyII0->text().toDouble();
  double desired_velocity = ui->LE_setvelocityII->text().toDouble();
  double endx = ui->LE_setxII1->text().toDouble();
  double endy = ui->LE_setyII1->text().toDouble();
  double endtheta = ui->LE_setthetaII1->text().toDouble();
  endtheta *= (M_PI / 180);
  // stop update setpoints
  globalvar::_threadloop.setsetpoint_second(startx, starty, desired_velocity,
                                            endx, endy, endtheta);
  globalvar::_threadloop.updatesetpoints_t();
  // ui operation
  ui->setupsetpointII->setEnabled(false);
  ui->resetsetpointII->setEnabled(true);

  ui->LE_setxII0->setEnabled(false);
  ui->LE_setyII0->setEnabled(false);
  ui->LE_setvelocityII->setEnabled(false);
  ui->LE_setxII1->setEnabled(false);
  ui->LE_setyII1->setEnabled(false);
  ui->LE_setthetaII1->setEnabled(false);
}

void Dialogstraightline::on_resetsetpointII_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->setupsetpointII->setEnabled(true);
  ui->resetsetpointII->setEnabled(false);
  ui->LE_setxII0->setEnabled(true);
  ui->LE_setyII0->setEnabled(true);
  ui->LE_setvelocityII->setEnabled(true);
  ui->LE_setxII1->setEnabled(true);
  ui->LE_setyII1->setEnabled(true);
  ui->LE_setthetaII1->setEnabled(true);
}

void Dialogstraightline::on_setupsetpointI_clicked() {
  double startx = ui->LE_setxI0->text().toDouble();
  double starty = ui->LE_setyI0->text().toDouble();
  double desired_velocity = ui->LE_setvelocityI->text().toDouble();
  double endx = ui->LE_setxI1->text().toDouble();
  double endy = ui->LE_setyI1->text().toDouble();
  double endtheta = ui->LE_setthetaI1->text().toDouble();
  endtheta *= (M_PI / 180);
  // stop update setpoints
  globalvar::_threadloop.setsetpoint_first(startx, starty, desired_velocity,
                                           endx, endy, endtheta);
  globalvar::_threadloop.updatesetpoints_t();
  // ui operation
  ui->setupsetpointI->setEnabled(false);
  ui->resetsetpointI->setEnabled(true);
  ui->LE_setxI0->setEnabled(false);
  ui->LE_setyI0->setEnabled(false);
  ui->LE_setvelocityI->setEnabled(false);
  ui->LE_setxI1->setEnabled(false);
  ui->LE_setyI1->setEnabled(false);
  ui->LE_setthetaI1->setEnabled(false);
}

void Dialogstraightline::on_resetsetpointI_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->setupsetpointI->setEnabled(true);
  ui->resetsetpointI->setEnabled(false);

  ui->LE_setxI0->setEnabled(true);
  ui->LE_setyI0->setEnabled(true);
  ui->LE_setvelocityI->setEnabled(true);
  ui->LE_setxI1->setEnabled(true);
  ui->LE_setyI1->setEnabled(true);
  ui->LE_setthetaI1->setEnabled(true);
}

void Dialogstraightline::initializeStraightlineDiag() {
  // the first straight line data
  strightlinedata data_first =
      globalvar::_threadloop.getstrightlinedata_first();

  QString valestring_first = QString::number(data_first.desired_initialx);
  ui->LE_setxI0->setText(valestring_first);
  valestring_first = QString::number(data_first.desired_initialy);
  ui->LE_setyI0->setText(valestring_first);
  valestring_first = QString::number(data_first.desired_velocity);
  ui->LE_setvelocityI->setText(valestring_first);

  valestring_first = QString::number(data_first.desired_finalx);
  ui->LE_setxI1->setText(valestring_first);
  valestring_first = QString::number(data_first.desired_finaly);
  ui->LE_setyI1->setText(valestring_first);
  valestring_first = QString::number(data_first.desired_theta);
  ui->LE_setthetaI1->setText(valestring_first);
  // the second straight line data
  strightlinedata data_second =
      globalvar::_threadloop.getstrightlinedata_second();

  QString valestring_second = QString::number(data_second.desired_initialx);
  ui->LE_setxII0->setText(valestring_second);
  valestring_second = QString::number(data_second.desired_initialy);
  ui->LE_setyII0->setText(valestring_second);
  valestring_second = QString::number(data_second.desired_velocity);
  ui->LE_setvelocityII->setText(valestring_second);

  valestring_second = QString::number(data_second.desired_finalx);
  ui->LE_setxII1->setText(valestring_second);
  valestring_second = QString::number(data_second.desired_finaly);
  ui->LE_setyII1->setText(valestring_second);
  valestring_second = QString::number(data_second.desired_theta);
  ui->LE_setthetaII1->setText(valestring_second);
}