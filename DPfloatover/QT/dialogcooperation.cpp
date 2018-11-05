#include "dialogcooperation.h"
#include "ui_dialogcooperation.h"

DialogCooperation::DialogCooperation(QWidget *parent)
    : QDialog(parent), ui(new Ui::DialogCooperation) {
  ui->setupUi(this);
  initilizeDialogCooperation();
}

DialogCooperation::~DialogCooperation() { delete ui; }

void DialogCooperation::initilizeDialogCooperation() {
  double desired_velocity = 0;
  double desired_theta = 0;
  double desired_initialx_first = 0;
  double desired_initialy_first = 0;
  double desired_finalx_first = 0;
  double desired_finaly_first = 0;
  double desired_initialx_second = 0;
  double desired_initialy_second = 0;
  double desired_finalx_second = 0;
  double desired_finaly_second = 0;
  globalvar::_threadloop.getstraightlinedata_both(
      desired_velocity, desired_theta, desired_initialx_first,
      desired_initialy_first, desired_finalx_first, desired_finaly_first,
      desired_initialx_second, desired_initialy_second, desired_finalx_second,
      desired_finaly_second);

  QString valestring = QString::number(desired_initialx_first);
  ui->LE_startxI->setText(valestring);
  valestring = QString::number(desired_initialy_first);
  ui->LE_startyI->setText(valestring);
  valestring = QString::number(desired_finalx_first);
  ui->LE_endxI->setText(valestring);
  valestring = QString::number(desired_finaly_first);
  ui->LE_endyI->setText(valestring);

  valestring = QString::number(desired_initialx_second);
  ui->LE_startxII->setText(valestring);
  valestring = QString::number(desired_initialy_second);
  ui->LE_startyII->setText(valestring);
  valestring = QString::number(desired_finalx_second);
  ui->LE_endxII->setText(valestring);
  valestring = QString::number(desired_finaly_second);
  ui->LE_endyII->setText(valestring);

  valestring = QString::number(desired_velocity);
  ui->LE_velocity->setText(valestring);
  valestring = QString::number(desired_theta);
  ui->LE_theta->setText(valestring);
}

void DialogCooperation::on_PB_setup_clicked() {
  double desired_initialx_first = ui->LE_startxI->text().toDouble();
  double desired_initialy_first = ui->LE_startyI->text().toDouble();
  double desired_finalx_first = ui->LE_endxI->text().toDouble();
  double desired_finaly_first = ui->LE_endyI->text().toDouble();
  double desired_initialx_second = ui->LE_startxII->text().toDouble();
  double desired_initialy_second = ui->LE_startyII->text().toDouble();
  double desired_finalx_second = ui->LE_endxII->text().toDouble();
  double desired_finaly_second = ui->LE_endyII->text().toDouble();
  double desired_velocity = ui->LE_velocity->text().toDouble();
  double desired_theta = ui->LE_theta->text().toDouble();
  desired_theta *= (M_PI / 180);
  globalvar::_threadloop.updatecooperationset_t(
      desired_velocity, desired_theta, desired_initialx_first,
      desired_initialy_first, desired_finalx_first, desired_finaly_first,
      desired_initialx_second, desired_initialy_second, desired_finalx_second,
      desired_finaly_second);

  // ui operation
  ui->PB_setup->setEnabled(false);
  ui->PB_reset->setEnabled(true);

  ui->LE_startxI->setEnabled(false);
  ui->LE_startyI->setEnabled(false);
  ui->LE_endxI->setEnabled(false);
  ui->LE_endyI->setEnabled(false);
  ui->LE_startxII->setEnabled(false);
  ui->LE_startyII->setEnabled(false);
  ui->LE_endxII->setEnabled(false);
  ui->LE_endyII->setEnabled(false);
  ui->LE_velocity->setEnabled(false);
  ui->LE_theta->setEnabled(false);
}

void DialogCooperation::on_PB_reset_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->PB_setup->setEnabled(true);
  ui->PB_reset->setEnabled(false);

  ui->LE_startxI->setEnabled(true);
  ui->LE_startyI->setEnabled(true);
  ui->LE_endxI->setEnabled(true);
  ui->LE_endyI->setEnabled(true);
  ui->LE_startxII->setEnabled(true);
  ui->LE_startyII->setEnabled(true);
  ui->LE_endxII->setEnabled(true);
  ui->LE_endyII->setEnabled(true);
  ui->LE_velocity->setEnabled(true);
  ui->LE_theta->setEnabled(true);
}
