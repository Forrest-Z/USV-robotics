#include "dialogcooperation.h"
#include "ui_dialogcooperation.h"

DialogCooperation::DialogCooperation(QWidget *parent)
    : QDialog(parent), ui(new Ui::DialogCooperation) {
  ui->setupUi(this);
  initilizeDialogCooperation();
}

DialogCooperation::~DialogCooperation() { delete ui; }

void DialogCooperation::initilizeDialogCooperation() {
  int total_time_spent = 0;
  double desired_initialx_first = 0;
  double desired_initialy_first = 0;
  double desired_initialtheta_first = 0;
  double delta_value_first = 0;
  int indicator_first = 0;
  double desired_initialx_second = 0;
  double desired_initialy_second = 0;
  double desired_initialtheta_second = 0;
  double delta_value_second = 0;
  int indicator_second = 0;
  double desired_initialx_third = 0;
  double desired_initialy_third = 0;
  double desired_initialtheta_third = 0;
  double delta_value_third = 0;
  int indicator_third = 0;

  globalvar::_threadloop.getstraightlinedata_triple(
      total_time_spent, desired_initialx_first, desired_initialy_first,
      desired_initialtheta_first, delta_value_first, indicator_first,
      desired_initialx_second, desired_initialy_second,
      desired_initialtheta_second, delta_value_second, indicator_second,
      desired_initialx_third, desired_initialy_third,
      desired_initialtheta_third, delta_value_third, indicator_third);

  QString valestring = QString::number(total_time_spent);
  ui->LE_time->setText(valestring);
  valestring = QString::number(desired_initialx_first);
  ui->LE_startxI->setText(valestring);
  valestring = QString::number(desired_initialy_first);
  ui->LE_startyI->setText(valestring);
  valestring = QString::number(desired_initialtheta_first);
  ui->LE_thetaI->setText(valestring);
  valestring = QString::number(delta_value_first);
  ui->LE_deltaI->setText(valestring);
  valestring = QString::number(indicator_first);
  ui->LE_indicatorI->setText(valestring);

  valestring = QString::number(desired_initialx_second);
  ui->LE_startxII->setText(valestring);
  valestring = QString::number(desired_initialy_second);
  ui->LE_startyII->setText(valestring);
  valestring = QString::number(desired_initialtheta_second);
  ui->LE_thetaII->setText(valestring);
  valestring = QString::number(delta_value_second);
  ui->LE_deltaII->setText(valestring);
  valestring = QString::number(indicator_second);
  ui->LE_indicatorII->setText(valestring);

  valestring = QString::number(desired_initialx_third);
  ui->LE_startxIII->setText(valestring);
  valestring = QString::number(desired_initialy_third);
  ui->LE_startyIII->setText(valestring);
  valestring = QString::number(desired_initialtheta_third);
  ui->LE_thetaIII->setText(valestring);
  valestring = QString::number(delta_value_third);
  ui->LE_deltaIII->setText(valestring);
  valestring = QString::number(indicator_third);
  ui->LE_indicatorIII->setText(valestring);
}

void DialogCooperation::on_PB_setup_clicked() {
  int total_time_spent = ui->LE_time->text().toInt();
  double desired_initialx_first = ui->LE_startxI->text().toDouble();
  double desired_initialy_first = ui->LE_startyI->text().toDouble();
  double desired_initialtheta_first =
      ui->LE_thetaI->text().toDouble() * (M_PI / 180);
  double delta_value_first = ui->LE_deltaI->text().toDouble();
  int indicator_first = ui->LE_indicatorI->text().toInt();
  double desired_initialx_second = ui->LE_startxII->text().toDouble();
  double desired_initialy_second = ui->LE_startyII->text().toDouble();
  double desired_initialtheta_second =
      ui->LE_thetaII->text().toDouble() * (M_PI / 180);
  double delta_value_second = ui->LE_deltaII->text().toDouble();
  int indicator_second = ui->LE_indicatorII->text().toInt();
  double desired_initialx_third = ui->LE_startxIII->text().toDouble();
  double desired_initialy_third = ui->LE_startyIII->text().toDouble();
  double desired_initialtheta_third =
      ui->LE_thetaIII->text().toDouble() * (M_PI / 180);
  double delta_value_third = ui->LE_deltaIII->text().toDouble();
  int indicator_third = ui->LE_indicatorIII->text().toInt();

  globalvar::_threadloop.updatecooperationset_t(
      total_time_spent, desired_initialx_first, desired_initialy_first,
      desired_initialtheta_first, delta_value_first, indicator_first,
      desired_initialx_second, desired_initialy_second,
      desired_initialtheta_second, delta_value_second, indicator_second,
      desired_initialx_third, desired_initialy_third,
      desired_initialtheta_third, delta_value_third, indicator_third);

  // ui operation
  ui->PB_getcurrentposition->setEnabled(false);
  ui->PB_setup->setEnabled(false);
  ui->PB_reset->setEnabled(true);

  ui->LE_time->setEnabled(false);

  ui->LE_startxI->setEnabled(false);
  ui->LE_startyI->setEnabled(false);
  ui->LE_thetaI->setEnabled(false);
  ui->LE_deltaI->setEnabled(false);
  ui->LE_indicatorI->setEnabled(false);

  ui->LE_startxII->setEnabled(false);
  ui->LE_startyII->setEnabled(false);
  ui->LE_thetaII->setEnabled(false);
  ui->LE_deltaII->setEnabled(false);
  ui->LE_indicatorII->setEnabled(false);

  ui->LE_startxIII->setEnabled(false);
  ui->LE_startyIII->setEnabled(false);
  ui->LE_thetaIII->setEnabled(false);
  ui->LE_deltaIII->setEnabled(false);
  ui->LE_indicatorIII->setEnabled(false);
}

void DialogCooperation::on_PB_reset_clicked() {
  // stop update setpoints
  globalvar::_threadloop.closeupdatesetpoints();
  // ui operation
  ui->PB_getcurrentposition->setEnabled(true);
  ui->PB_setup->setEnabled(true);
  ui->PB_reset->setEnabled(false);

  ui->LE_time->setEnabled(true);

  ui->LE_startxI->setEnabled(true);
  ui->LE_startyI->setEnabled(true);
  ui->LE_thetaI->setEnabled(true);
  ui->LE_deltaI->setEnabled(true);
  ui->LE_indicatorI->setEnabled(true);

  ui->LE_startxII->setEnabled(true);
  ui->LE_startyII->setEnabled(true);
  ui->LE_thetaII->setEnabled(true);
  ui->LE_deltaII->setEnabled(true);
  ui->LE_indicatorII->setEnabled(true);

  ui->LE_startxIII->setEnabled(true);
  ui->LE_startyIII->setEnabled(true);
  ui->LE_thetaIII->setEnabled(true);
  ui->LE_deltaIII->setEnabled(true);
  ui->LE_indicatorIII->setEnabled(true);
}

void DialogCooperation::on_PB_getcurrentposition_clicked() {
  Eigen::Vector3d _setpoints = globalvar::_threadloop.getSetpoints_first();
  QString valestring = QString::number(_setpoints(0));
  ui->LE_startxI->setText(valestring);
  valestring = QString::number(_setpoints(1));
  ui->LE_startyI->setText(valestring);
  valestring = QString::number(_setpoints(2) * 180 / M_PI);
  ui->LE_thetaI->setText(valestring);
  _setpoints = globalvar::_threadloop.getSetpoints_second();
  valestring = QString::number(_setpoints(0));
  ui->LE_startxII->setText(valestring);
  valestring = QString::number(_setpoints(1));
  ui->LE_startyII->setText(valestring);
  valestring = QString::number(_setpoints(2) * 180 / M_PI);
  ui->LE_thetaII->setText(valestring);
  _setpoints = globalvar::_threadloop.getSetpoints_third();
  valestring = QString::number(_setpoints(0));
  ui->LE_startxIII->setText(valestring);
  valestring = QString::number(_setpoints(1));
  ui->LE_startyIII->setText(valestring);
  valestring = QString::number(_setpoints(2) * 180 / M_PI);
  ui->LE_thetaIII->setText(valestring);
}
